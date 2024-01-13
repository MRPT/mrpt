/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "math-precomp.h"  // Precompiled headers
//
#include <mrpt/core/optional_ref.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CPolygon.h>
#include <mrpt/math/CSparseMatrixTemplate.h>
#include <mrpt/math/CVectorFixed.h>
#include <mrpt/math/TLine2D.h>
#include <mrpt/math/TLine3D.h>
#include <mrpt/math/TObject2D.h>
#include <mrpt/math/TObject3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/data_utils.h>
#include <mrpt/math/geometry.h>
#include <mrpt/math/ops_containers.h>

#include <Eigen/Dense>

using namespace mrpt;
using namespace std;
using namespace mrpt::math;

static double geometryEpsilon = 1e-5;

double mrpt::math::getEpsilon() { return geometryEpsilon; }
void mrpt::math::setEpsilon(double eps) { geometryEpsilon = eps; }
/*---------------------------------------------------------------
	Returns the closest point to a segment
  ---------------------------------------------------------------*/
void math::closestFromPointToSegment(
	double Px, double Py, double x1, double y1, double x2, double y2,
	double& out_x, double& out_y)
{
	if (x1 == x2 && y1 == y2)
	{
		out_x = x1;
		out_y = y1;
	}
	else
	{
		double Dx = x2 - x1;
		double Dy = y2 - y1;
		double Ratio = ((Px - x1) * Dx + (Py - y1) * Dy) / (Dx * Dx + Dy * Dy);
		if (Ratio < 0)
		{
			out_x = x1;
			out_y = y1;
		}
		else
		{
			if (Ratio > 1)
			{
				out_x = x2;
				out_y = y2;
			}
			else
			{
				out_x = x1 + (Ratio * Dx);
				out_y = y1 + (Ratio * Dy);
			}
		}
	}
}

/*---------------------------------------------------------------
	Returns the closest point to a line
  ---------------------------------------------------------------*/
void math::closestFromPointToLine(
	double Px, double Py, double x1, double y1, double x2, double y2,
	double& out_x, double& out_y)
{
	if (x1 == x2 && y1 == y2)
	{
		out_x = x1;
		out_y = y1;
	}
	else
	{
		double Dx = x2 - x1;
		double Dy = y2 - y1;
		double Ratio = ((Px - x1) * Dx + (Py - y1) * Dy) / (Dx * Dx + Dy * Dy);

		out_x = x1 + (Ratio * Dx);
		out_y = y1 + (Ratio * Dy);
	}
}

/*---------------------------------------------------------------
	Returns the sq. distance to closest point to a line
  ---------------------------------------------------------------*/
double math::squaredDistancePointToLine(
	double Px, double Py, double x1, double y1, double x2, double y2)
{
	if (x1 == x2 && y1 == y2) { return square(Px - x1) + square(Py - y1); }
	else
	{
		double Dx = x2 - x1;
		double Dy = y2 - y1;
		double Ratio = ((Px - x1) * Dx + (Py - y1) * Dy) / (Dx * Dx + Dy * Dy);

		return square(x1 + (Ratio * Dx) - Px) + square(y1 + (Ratio * Dy) - Py);
	}
}

/*---------------------------------------------------------------
				Rectangles Intersect
  ---------------------------------------------------------------*/
bool math::RectanglesIntersection(
	double R1_x_min, double R1_x_max, double R1_y_min, double R1_y_max,
	double R2_x_min, double R2_x_max, double R2_y_min, double R2_y_max,
	double R2_pose_x, double R2_pose_y, double R2_pose_phi)
{
	// Compute the rotated R2:
	// ----------------------------------------
	CVectorDouble xs(4), ys(4);
	double ccos = cos(R2_pose_phi);
	double ssin = sin(R2_pose_phi);

	xs[0] = R2_pose_x + ccos * R2_x_min - ssin * R2_y_min;
	ys[0] = R2_pose_y + ssin * R2_x_min + ccos * R2_y_min;

	xs[1] = R2_pose_x + ccos * R2_x_max - ssin * R2_y_min;
	ys[1] = R2_pose_y + ssin * R2_x_max + ccos * R2_y_min;

	xs[2] = R2_pose_x + ccos * R2_x_max - ssin * R2_y_max;
	ys[2] = R2_pose_y + ssin * R2_x_max + ccos * R2_y_max;

	xs[3] = R2_pose_x + ccos * R2_x_min - ssin * R2_y_max;
	ys[3] = R2_pose_y + ssin * R2_x_min + ccos * R2_y_max;

	// Test for one vertice being inside the other rectangle:
	// -------------------------------------------------------
	if (R1_x_min <= xs[0] && xs[0] <= R1_x_max && R1_y_min <= ys[0] &&
		ys[0] <= R1_y_max)
		return true;
	if (R1_x_min <= xs[1] && xs[1] <= R1_x_max && R1_y_min <= ys[1] &&
		ys[1] <= R1_y_max)
		return true;
	if (R1_x_min <= xs[2] && xs[2] <= R1_x_max && R1_y_min <= ys[2] &&
		ys[2] <= R1_y_max)
		return true;
	if (R1_x_min <= xs[3] && xs[3] <= R1_x_max && R1_y_min <= ys[3] &&
		ys[3] <= R1_y_max)
		return true;

	CPolygon poly;
	poly.AddVertex(xs[0], ys[0]);
	poly.AddVertex(xs[1], ys[1]);
	poly.AddVertex(xs[2], ys[2]);
	poly.AddVertex(xs[3], ys[3]);

	if (poly.PointIntoPolygon(R1_x_min, R1_y_min)) return true;
	if (poly.PointIntoPolygon(R1_x_max, R1_y_min)) return true;
	if (poly.PointIntoPolygon(R1_x_max, R1_y_max)) return true;
	if (poly.PointIntoPolygon(R1_x_min, R1_y_max)) return true;

	// Test for intersections:
	// ----------------------------------------
	TObject2D intersectObj;

	for (int idx = 0; idx < 4; idx++)
	{
		const auto seg2 = TSegment2D::FromPoints(
			{xs[idx], ys[idx]}, {xs[(idx + 1) % 4], ys[(idx + 1) % 4]});

		if (mrpt::math::intersect(
				TSegment2D::FromPoints(
					{R1_x_min, R1_y_min}, {R1_x_max, R1_y_min}),
				seg2, intersectObj))
			return true;
		if (mrpt::math::intersect(
				TSegment2D::FromPoints(
					{R1_x_max, R1_y_min}, {R1_x_max, R1_y_max}),
				seg2, intersectObj))
			return true;
		if (mrpt::math::intersect(
				TSegment2D::FromPoints(
					{R1_x_max, R1_y_max}, {R1_x_min, R1_y_max}),
				seg2, intersectObj))
			return true;
		if (mrpt::math::intersect(
				TSegment2D::FromPoints(
					{R1_x_min, R1_y_max}, {R1_x_min, R1_y_min}),
				seg2, intersectObj))
			return true;
	}

	// No intersections:
	return false;
}

// Auxiliary functions needed to avoid code repetition and unnecesary
// recalculations
template <class T2D, class U2D, class T3D, class U3D>
bool intersectInCommonPlane(
	const T3D& o1, const U3D& o2, const mrpt::math::TPlane& p,
	mrpt::math::TObject3D& obj)
{
	T3D proj1;
	U3D proj2;
	// Project into 3D plane, ignoring Z coordinate.
	TPose3D pose;
	TPlane(p).getAsPose3D(pose);
	TPose3D poseNeg = -pose;
	project3D(o1, poseNeg, proj1);
	project3D(o2, poseNeg, proj2);
	T2D proj1_2D;
	U2D proj2_2D;
	proj1.generate2DObject(proj1_2D);
	proj2.generate2DObject(proj2_2D);
	// Compute easier intersection in 2D space
	TObject2D obj2D;
	if (intersect(proj1_2D, proj2_2D, obj2D))
	{
		TObject3D tmp = obj2D.generate3DObject();
		// Undo projection
		project3D(tmp, pose, obj);
		return true;
	}
	else
		return false;
}
bool intersectInCommonLine(
	const mrpt::math::TSegment3D& s1, const mrpt::math::TSegment3D& s2,
	const mrpt::math::TLine3D& lin, mrpt::math::TObject3D& obj)
{
	// Move in a free coordinate, searching for minima and maxima.
	size_t i1 = 0;
	while (std::abs(lin.director[i1]) < geometryEpsilon)
		i1++;
	TSegment3D s11 = (s1[0][i1] > s1[1][i1]) ? TSegment3D(s1[1], s1[0]) : s1;
	TSegment3D s21 = (s2[0][i1] > s2[1][i1]) ? TSegment3D(s2[1], s2[0]) : s2;
	TPoint3D pMin = ((s11[0][i1] < s21[0][i1]) ? s21 : s11)[0];
	TPoint3D pMax = ((s11[1][i1] < s21[1][i1]) ? s11 : s21)[1];
	if (std::abs(pMax[i1] - pMin[i1]) < geometryEpsilon)
	{  // Intersection is a point
		obj.data = pMax;
		return true;
	}
	else if (pMax[i1] < pMin[i1])
		return false;  // No intersection
	else
	{
		obj.data = TSegment3D(pMin, pMax);	// Intersection is a segment
		return true;
	}
}
bool intersectInCommonLine(
	const TSegment2D& s1, const TSegment2D& s2, const TLine2D& lin,
	TObject2D& obj)
{
	// Move in a free coordinate, searching for minima and maxima
	size_t i1 = (std::abs(lin.coefs[0]) >= geometryEpsilon) ? 1 : 0;
	TSegment2D s11 = (s1[0][i1] > s1[1][i1]) ? TSegment2D(s1[1], s1[0]) : s1;
	TSegment2D s21 = (s2[0][i1] > s2[1][i1]) ? TSegment2D(s2[1], s2[0]) : s2;
	TPoint2D pMin = ((s11[0][i1] < s21[0][i1]) ? s21 : s11)[0];
	TPoint2D pMax = ((s11[1][i1] < s21[1][i1]) ? s11 : s21)[1];
	if (std::abs(pMax[i1] - pMin[i1]) < geometryEpsilon)
	{  // Intersection is a point
		obj.data = pMax;
		return true;
	}
	else if (pMax[i1] < pMin[i1])
		return false;  // No intersection
	else
	{
		obj.data = TSegment2D(pMin, pMax);	// Intersection is a segment
		return true;
	}
}
inline void unsafeProjectPoint(
	const TPoint3D& point, const TPose3D& pose, TPoint2D& newPoint)
{
	TPoint3D dummy;
	pose.composePoint(point, dummy);
	newPoint.x = dummy.x;
	newPoint.y = dummy.y;
}
void mrpt::math::internal::unsafeProjectPolygon(
	const TPolygon3D& poly, const TPose3D& pose, TPolygon2D& newPoly)
{
	size_t N = poly.size();
	newPoly.resize(N);
	for (size_t i = 0; i < N; i++)
		unsafeProjectPoint(poly[i], pose, newPoly[i]);
}
bool intersect(
	const TPolygonWithPlane& p1, const TLine3D& l2, double& d, double bestKnown)
{
	// LINE MUST BE UNITARY
	TObject3D obj;
	if (intersect(p1.plane, l2, obj))
		if (obj.isPoint())
		{
			const TPoint3D p = obj.getAs<TPoint3D>();
			for (size_t i = 0; i < 3; i++)
				if (std::abs(l2.director[i]) > geometryEpsilon)
				{
					d = (p[i] - l2.pBase[i]) / l2.director[i];
					break;
				}
			if (d < 0 || d > bestKnown) return false;
			TPolygon2D newPoly;
			TPoint2D newP;
			unsafeProjectPoint(p, p1.inversePose, newP);
			mrpt::math::internal::unsafeProjectPolygon(
				p1.poly, p1.inversePose, newPoly);
			return newPoly.contains(newP);
		}
	return false;
}
bool intersect(
	const TPolygonWithPlane& p1, const TPolygonWithPlane& p2, TObject3D& obj)
{
	if (!intersect(p1.plane, p2.plane, obj)) return false;

	TObject3D aux;
	if (obj.isLine())
	{
		const TLine3D lin3D = obj.getAs<TLine3D>();
		TLine3D lin3D1, lin3D2;
		TLine2D lin2D1, lin2D2;
		TObject2D obj2D1, obj2D2;
		project3D(lin3D, p1.inversePose, lin3D1);
		project3D(lin3D, p2.inversePose, lin3D2);
		lin3D1.generate2DObject(lin2D1);
		lin3D2.generate2DObject(lin2D2);
		if (intersect(p1.poly2D, lin2D1, obj2D1) &&
			intersect(p2.poly2D, lin2D2, obj2D2))
		{
			TObject3D obj3Dp1, obj3Dp2;
			const TObject3D obj3D1 = obj2D1.generate3DObject();
			const TObject3D obj3D2 = obj2D2.generate3DObject();
			project3D(obj3D1, p1.pose, obj3Dp1);
			project3D(obj3D2, p2.pose, obj3Dp2);
			TSegment3D s1, s2;
			if (obj3D1.isPoint())
			{
				const auto po1 = obj3D1.getAs<TPoint3D>();
				s1 = TSegment3D(po1, po1);
			}
			else
			{
				s1 = obj3D1.getAs<TSegment3D>();
			}
			TPoint3D po2;
			if (obj3D2.getPoint(po2)) s2 = TSegment3D(po2, po2);
			else
				obj3D2.getSegment(s2);
			return intersectInCommonLine(s1, s2, lin3D, obj);
		}
		else
			return false;
	}
	else
	{
		TObject2D obj2D;
		if (intersect(p1.poly2D, p2.poly2D, obj2D))
		{
			aux = obj2D.generate3DObject();
			project3D(aux, p1.pose, obj);
			return true;
		}
		else
			return false;
	}
}
// End of auxiliary methods

bool math::intersect(const TSegment3D& s1, const TSegment3D& s2, TObject3D& obj)
{
	TObject3D irr;
	auto l = TLine3D(s1);
	if (!intersect(l, TLine3D(s2), irr)) return false;
	if (irr.isPoint())
	{
		// Both lines cross in a point.
		TPoint3D p;
		irr.getPoint(p);
		if (s1.contains(p) && s2.contains(p))
		{
			obj.data = p;
			return true;
		}
		else
			return false;
	}
	else
		return intersectInCommonLine(s1, s2, l, obj);
}

bool math::intersect(const TSegment3D& s1, const TPlane& p1, TObject3D& obj)
{
	if (!intersect(TLine3D(s1), p1, obj)) return false;
	if (obj.isLine())
	{
		// Segment is fully inside the plane, so it is the return value.
		obj.data = s1;
		return true;
	}
	else
	{
		// Segment's line intersects the plane in a point. This may be or not be
		// part of the segment.
		TPoint3D p;
		if (!obj.getPoint(p)) return false;
		return s1.contains(p);
	}
}

bool math::intersect(const TSegment3D& s1, const TLine3D& r1, TObject3D& obj)
{
	if (!intersect(TLine3D(s1), r1, obj)) return false;
	if (obj.isLine())
	{
		// Segment's line is the other line.
		obj.data = s1;
		return true;
	}
	else
	{
		// Segment's line and the other line cross in a point, which may be or
		// not be inside the segment.
		TPoint3D p;
		if (!obj.getPoint(p)) return false;
		return s1.contains(p);
	}
}

bool math::intersect(const TPlane& p1, const TPlane& p2, TObject3D& obj)
{
	TLine3D lin;
	crossProduct3D(p1.coefs, p2.coefs, lin.director);
	if ((std::abs(lin.director[0]) < geometryEpsilon) &&
		(std::abs(lin.director[1]) < geometryEpsilon) &&
		(std::abs(lin.director[2]) < geometryEpsilon))
	{
		// Planes are parallel
		for (size_t i = 0; i < 3; i++)
			if (std::abs(
					p1.coefs[i] * p2.coefs[3] - p1.coefs[3] * p2.coefs[i]) >=
				geometryEpsilon)
				return false;
		// Planes are the same
		obj.data = p1;
		return true;
	}
	else
	{
		// Planes cross in a line whose director vector is already calculated
		// (normal to both planes' normal).
		// The following process manages to create a random point in the line
		// without loss of generality and almost without conditional sentences.
		size_t i1 = 0;
		while (std::abs(lin.director[i1]) < geometryEpsilon)
			i1++;
		// At this point, i1 points to a coordinate (0->x, 1->y, 2->z) in which
		// we can move freely.
		// If we arbitrarily assign this coordinate to 0, we'll find a suitable
		// base point by solving both planes' equations.
		size_t c1 = (i1 + 1) % 3, c2 = (i1 + 2) % 3;
		lin.pBase[i1] = 0.0;
		lin.pBase[c1] =
			(p2.coefs[3] * p1.coefs[c2] - p1.coefs[3] * p2.coefs[c2]) /
			lin.director[i1];
		lin.pBase[c2] =
			(p2.coefs[c1] * p1.coefs[3] - p1.coefs[c1] * p2.coefs[3]) /
			lin.director[i1];
		lin.unitarize();
		obj.data = lin;
		return true;
	}
}

bool math::intersect(const TPlane& p1, const TLine3D& r2, TObject3D& obj)
{
	// double
	// n=p1.coefs[0]*r2.director[0]+p1.coefs[1]*r2.director[1]+p1.coefs[2]*r2.director[2];
	double n = dotProduct<3, double>(p1.coefs, r2.director);
	double e = p1.evaluatePoint(r2.pBase);
	if (std::abs(n) < geometryEpsilon)
	{
		// Plane's normal and line's director are orthogonal, so both are
		// parallel
		if (std::abs(e) < geometryEpsilon)
		{
			// Line is contained in plane.
			obj.data = r2;
			return true;
		}
		else
			return false;
	}
	else
	{
		// Plane and line cross in a point.
		double t = e / n;
		TPoint3D p;
		p.x = r2.pBase.x - t * r2.director[0];
		p.y = r2.pBase.y - t * r2.director[1];
		p.z = r2.pBase.z - t * r2.director[2];
		obj.data = p;
		return true;
	}
}

bool math::intersect(const TLine3D& r1, const TLine3D& r2, TObject3D& obj)
{
	double u, d[3];
	TPoint3D p;
	const static size_t c1[] = {1, 2, 0};
	const static size_t c2[] = {2, 0, 1};
	// With indirect memory accesses, almost all the code goes in a single loop.
	for (size_t i = 0; i < 3; i++)
	{
		double sysDet = -r1.director[c1[i]] * r2.director[c2[i]] +
			r2.director[c1[i]] * r1.director[c2[i]];
		if (std::abs(sysDet) < geometryEpsilon) continue;
		// We've found a coordinate in which we can solve the associated system
		d[c1[i]] = r2.pBase[c1[i]] - r1.pBase[c1[i]];
		d[c2[i]] = r2.pBase[c2[i]] - r1.pBase[c2[i]];
		u = (r1.director[c1[i]] * d[c2[i]] - r1.director[c2[i]] * d[c1[i]]) /
			sysDet;
		for (size_t k = 0; k < 3; k++)
			p[k] = r2.pBase[k] + u * r2.director[k];
		if (r1.contains(p))
		{
			obj.data = p;
			return true;
		}
		else
			return false;
	}
	// Lines are parallel
	if (r1.contains(r2.pBase))
	{
		// Lines are the same
		obj.data = r1;
		return true;
	}
	else
		return false;
}

bool math::intersect(const TLine2D& r1, const TLine2D& r2, TObject2D& obj)
{
	double sysDet = r1.coefs[0] * r2.coefs[1] - r1.coefs[1] * r2.coefs[0];
	if (std::abs(sysDet) >= geometryEpsilon)
	{
		// Resulting point comes simply from solving an equation.
		TPoint2D p;
		p.x = (r1.coefs[1] * r2.coefs[2] - r1.coefs[2] * r2.coefs[1]) / sysDet;
		p.y = (r1.coefs[2] * r2.coefs[0] - r1.coefs[0] * r2.coefs[2]) / sysDet;
		obj.data = p;
		return true;
	}
	else
	{
		// Lines are parallel
		if (std::abs(r1.coefs[0] * r2.coefs[2] - r1.coefs[2] * r2.coefs[0]) >=
				geometryEpsilon ||
			std::abs(r1.coefs[1] * r2.coefs[2] - r1.coefs[2] * r2.coefs[1]) >=
				geometryEpsilon)
			return false;

		// Lines are the same
		obj.data = r1;
		return true;
	}
}

bool math::intersect(const TLine2D& r1, const TSegment2D& s2, TObject2D& obj)
{
	if (!intersect(r1, TLine2D(s2), obj)) return false;
	TPoint2D p;
	if (obj.isLine())
	{
		// Segment is inside the line
		obj.data = s2;
		return true;
	}
	else if (obj.getPoint(p))
		return s2.contains(p);	// Both lines cross in a point.
	return false;
}

bool math::intersect(const TSegment2D& s1, const TSegment2D& s2, TObject2D& obj)
{
	auto lin = TLine2D(s1);
	if (!intersect(lin, TLine2D(s2), obj)) return false;
	TPoint2D p;
	if (obj.isLine())
		return intersectInCommonLine(
			s1, s2, lin, obj);	// Segments' lines are parallel
	else if (obj.getPoint(p))
		return s1.contains(p) &&
			s2.contains(p);	 // Segments' lines cross in a point
	return false;
}

double math::getAngle(const TPlane& s1, const TPlane& s2)
{
	double c = 0, n1 = 0, n2 = 0;
	for (size_t i = 0; i < 3; i++)
	{
		c += s1.coefs[i] * s2.coefs[i];
		n1 += s1.coefs[i] * s1.coefs[i];
		n2 += s2.coefs[i] * s2.coefs[i];
	}
	double s = sqrt(n1 * n2);
	if (s < geometryEpsilon) THROW_EXCEPTION("Invalid plane(s)");
	if (std::abs(s) < std::abs(c)) return (c > 0) ? 0 : M_PI;
	else
		return acos(c / s);
}

double math::getAngle(const TPlane& s1, const TLine3D& r2)
{
	double c = 0, n1 = 0, n2 = 0;
	for (size_t i = 0; i < 3; i++)
	{
		c += s1.coefs[i] * r2.director[i];
		n1 += s1.coefs[i] * s1.coefs[i];
		n2 += r2.director[i] * r2.director[i];
	}
	double s = sqrt(n1 * n2);
	if (s < geometryEpsilon) THROW_EXCEPTION("Invalid plane or line");
	if (std::abs(s) < std::abs(c)) return (c > 0) ? 0 : M_PI;
	else
		return asin(c / s);
}

double math::getAngle(const TLine3D& r1, const TLine3D& r2)
{
	double c = 0, n1 = 0, n2 = 0;
	for (size_t i = 0; i < 3; i++)
	{
		c += r1.director[i] * r2.director[i];
		n1 += r1.director[i] * r1.director[i];
		n2 += r2.director[i] * r2.director[i];
	}
	double s = sqrt(n1 * n2);
	if (s < geometryEpsilon) THROW_EXCEPTION("Invalid line(s)");
	if (std::abs(s) < std::abs(c)) return (c / s < 0) ? M_PI : 0;
	else
		return acos(c / s);
}

double math::getAngle(const TLine2D& r1, const TLine2D& r2)
{
	const double ang1 = std::atan2(-r1.coefs[0], r1.coefs[1]);
	const double ang2 = std::atan2(-r2.coefs[0], r2.coefs[1]);
	return mrpt::math::wrapToPi(ang2 - ang1);
}

// Auxiliary method
void createFromPoseAndAxis(const TPose3D& p, TLine3D& r, size_t axis)
{
	CMatrixDouble44 m;
	p.getHomogeneousMatrix(m);
	for (size_t i = 0; i < 3; i++)
	{
		r.pBase[i] = m(i, 3);
		r.director[i] = m(i, axis);
	}
}
// End of auxiliary method

void math::createFromPoseX(const TPose3D& p, TLine3D& r)
{
	createFromPoseAndAxis(p, r, 0);
}

void math::createFromPoseY(const TPose3D& p, TLine3D& r)
{
	createFromPoseAndAxis(p, r, 1);
}

void math::createFromPoseZ(const TPose3D& p, TLine3D& r)
{
	createFromPoseAndAxis(p, r, 2);
}

void math::createFromPoseAndVector(
	const TPose3D& p, const double (&vector)[3], TLine3D& r)
{
	CMatrixDouble44 m;
	p.getHomogeneousMatrix(m);
	for (size_t i = 0; i < 3; i++)
	{
		r.pBase[i] = m(i, 3);
		r.director[i] = 0;
		for (size_t j = 0; j < 3; j++)
			r.director[i] += m(i, j) * vector[j];
	}
}

void math::createFromPoseX(const TPose2D& p, TLine2D& r)
{
	r.coefs[0] = cos(p.phi);
	r.coefs[1] = -sin(p.phi);
	r.coefs[2] = -r.coefs[0] * p.x - r.coefs[1] * p.y;
}

void math::createFromPoseY(const TPose2D& p, TLine2D& r)
{
	r.coefs[0] = sin(p.phi);
	r.coefs[1] = cos(p.phi);
	r.coefs[2] = -r.coefs[0] * p.x - r.coefs[1] * p.y;
}

void math::createFromPoseAndVector(
	const TPose2D& p, const double (&vector)[2], TLine2D& r)
{
	double c = cos(p.phi);
	double s = sin(p.phi);
	r.coefs[0] = vector[0] * c + vector[1] * s;
	r.coefs[1] = -vector[0] * s + vector[1] * c;
	r.coefs[2] = -r.coefs[0] * p.x - r.coefs[1] * p.y;
}

bool math::conformAPlane(const std::vector<TPoint3D>& points)
{
	size_t N = points.size();
	if (N < 3) return false;
	CMatrixDouble mat(N - 1, 3);
	const TPoint3D& orig = points[N - 1];
	for (size_t i = 0; i < N - 1; i++)
	{
		const TPoint3D& p = points[i];
		mat(i, 0) = p.x - orig.x;
		mat(i, 1) = p.y - orig.y;
		mat(i, 2) = p.z - orig.z;
	}
	return mat.rank(geometryEpsilon) == 2;
}

bool math::conformAPlane(const std::vector<TPoint3D>& points, TPlane& p)
{
	return std::abs(getRegressionPlane(points, p)) < geometryEpsilon;
}

bool math::areAligned(const std::vector<TPoint2D>& points)
{
	size_t N = points.size();
	if (N < 2) return false;
	CMatrixDouble mat(N - 1, 2);
	const TPoint2D& orig = points[N - 1];
	for (size_t i = 0; i < N - 1; i++)
	{
		const TPoint2D& p = points[i];
		mat(i, 0) = p.x - orig.x;
		mat(i, 1) = p.y - orig.y;
	}
	return mat.rank(geometryEpsilon) == 1;
}

bool math::areAligned(const std::vector<TPoint2D>& points, TLine2D& r)
{
	if (!areAligned(points)) return false;
	const TPoint2D& p0 = points[0];
	for (size_t i = 1;; i++)
	{
		try
		{
			r = TLine2D(p0, points[i]);
			return true;
		}
		catch (logic_error&)
		{
		}
	}
}

bool math::areAligned(const std::vector<TPoint3D>& points)
{
	size_t N = points.size();
	if (N < 2) return false;
	CMatrixDouble mat(N - 1, 3);
	const TPoint3D& orig = points[N - 1];
	for (size_t i = 0; i < N - 1; i++)
	{
		const TPoint3D& p = points[i];
		mat(i, 0) = p.x - orig.x;
		mat(i, 1) = p.y - orig.y;
		mat(i, 2) = p.z - orig.z;
	}
	return mat.rank(geometryEpsilon) == 1;
}

bool math::areAligned(const std::vector<TPoint3D>& points, TLine3D& r)
{
	if (!areAligned(points)) return false;
	const TPoint3D& p0 = points[0];
	for (size_t i = 1;; i++)
		try
		{
			r = TLine3D(p0, points[i]);
			return true;
		}
		catch (logic_error&)
		{
		}
}

void math::project3D(
	const TLine3D& line, const TPose3D& newXYpose, TLine3D& newLine)
{
	newXYpose.composePoint(line.pBase, newLine.pBase);
	CMatrixDouble44 mat;
	newXYpose.getHomogeneousMatrix(mat);
	for (size_t i = 0; i < 3; i++)
	{
		newLine.director[i] = 0;
		for (size_t j = 0; j < 3; j++)
			newLine.director[i] += mat(i, j) * line.director[j];
	}
	newLine.unitarize();
}

void math::project3D(
	const TPlane& plane, const TPose3D& newXYpose, TPlane& newPlane)
{
	CMatrixDouble44 mat;
	newXYpose.getHomogeneousMatrix(mat);
	for (size_t i = 0; i < 3; i++)
	{
		newPlane.coefs[i] = 0;
		for (size_t j = 0; j < 3; j++)
			newPlane.coefs[i] += mat(i, j) * plane.coefs[j];
	}
	// VORSICHT! NO INTENTEN HACER ESTO EN SUS CASAS (nota: comentar sí o sí,
	// más tarde)
	// La idea es mantener la distancia al nuevo origen igual a la distancia del
	// punto original antes de proyectar.
	// newPlane.coefs[3]=plane.evaluatePoint(TPoint3D(TPose3D(0,0,0,0,0,0)-newXYpose))*sqrt((newPlane.coefs[0]*newPlane.coefs[0]+newPlane.coefs[1]*newPlane.coefs[1]+newPlane.coefs[2]*newPlane.coefs[2])/(plane.coefs[0]*plane.coefs[0]+plane.coefs[1]*plane.coefs[1]+plane.coefs[2]*plane.coefs[2]));
	CMatrixDouble44 HMinv;
	newXYpose.getInverseHomogeneousMatrix(HMinv);
	newPlane.coefs[3] = plane.evaluatePoint(
							TPoint3D(HMinv(0, 3), HMinv(1, 3), HMinv(2, 3))) *
		sqrt(squareNorm<3, double>(newPlane.coefs) /
			 squareNorm<3, double>(plane.coefs));
	newPlane.unitarize();
}

namespace mrpt::math
{
// dummy, required for std::visit()
void project3D(const std::monostate&, const TPose3D&, std::monostate&) {}
void project2D(const std::monostate&, const TPose3D&, std::monostate&) {}
}  // namespace mrpt::math

void math::project3D(
	const TPolygon3D& polygon, const TPose3D& newXYpose, TPolygon3D& newPolygon)
{
	size_t N = polygon.size();
	newPolygon.resize(N);
	for (size_t i = 0; i < N; i++)
		project3D(polygon[i], newXYpose, newPolygon[i]);
}

void math::project3D(
	const TObject3D& object, const TPose3D& newXYpose, TObject3D& newObject)
{
	std::visit(
		[&](auto& o) {
			using T = std::decay_t<decltype(o)>;
			newObject.data.emplace<T>();
			project3D(o, newXYpose, std::get<T>(newObject.data));
		},
		object.data);
}

void math::project2D(
	const TPoint2D& point, const TPose2D& newXpose, TPoint2D& newPoint)
{
	newPoint = newXpose.composePoint(point);
}

void math::project2D(
	const TLine2D& line, const TPose2D& newXpose, TLine2D& newLine)
{
	double c = cos(newXpose.phi);
	double s = sin(newXpose.phi);
	newLine.coefs[0] = line.coefs[0] * c - line.coefs[1] * s;
	newLine.coefs[1] = line.coefs[1] * c + line.coefs[0] * s;
	newLine.coefs[2] = line.coefs[2] -
		(newLine.coefs[0] * newXpose.x + newLine.coefs[1] * newXpose.y);
	return;
}

void math::project2D(
	const TPolygon2D& line, const TPose2D& newXpose, TPolygon2D& newLine)
{
	size_t N = line.size();
	newLine.resize(N);
	for (size_t i = 0; i < N; i++)
		newLine[i] = newXpose + line[i];
	return;
}

void math::project2D(
	const TObject2D& obj, const TPose2D& newXpose, TObject2D& newObject)
{
	std::visit(
		[&](auto& o) {
			using T = std::decay_t<decltype(o)>;
			newObject.data.emplace<T>();
			project2D(o, newXpose, std::get<T>(newObject.data));
		},
		obj.data);
}

bool math::intersect(const TPolygon2D& p1, const TSegment2D& s2, TObject2D& obj)
{
	auto l2 = TLine2D(s2);
	if (!intersect(p1, l2, obj)) return false;
	TPoint2D p;
	TSegment2D s;
	if (obj.getPoint(p)) return s2.contains(p);
	else if (obj.getSegment(s))
		return intersectInCommonLine(s, s2, l2, obj);
	return false;
}

bool math::intersect(const TPolygon2D& p1, const TLine2D& r2, TObject2D& obj)
{
	// Proceeding: project polygon so that the line happens to be y=0 (phi=0).
	// Then, search this new polygons for intersections with the X axis (very
	// simple).
	if (p1.size() < 3) return false;
	TPose2D pose, poseNeg;
	r2.getAsPose2D(pose);
	poseNeg = TPose2D(0, 0, 0) - pose;
	TPolygon2D projPoly;
	project2D(p1, poseNeg, projPoly);
	size_t N = projPoly.size();
	projPoly.push_back(projPoly[0]);
	double pre = projPoly[0].y;
	vector<TPoint2D> pnts;
	pnts.reserve(2);
	for (size_t i = 1; i <= N; i++)
	{
		double cur = projPoly[i].y;
		if (std::abs(cur) < geometryEpsilon)
		{
			if (std::abs(pre) < geometryEpsilon)
			{
				pnts.resize(2);
				pnts[0] = projPoly[i - 1];
				pnts[1] = projPoly[i];
				break;
			}
			else
				pnts.push_back(projPoly[i]);
		}
		else if ((std::abs(pre) >= geometryEpsilon) && (sign(cur) != sign(pre)))
		{
			double a = projPoly[i - 1].x;
			double c = projPoly[i].x;
			double x = a - pre * (c - a) / (cur - pre);
			pnts.emplace_back(x, 0);
		}
		pre = cur;
	}
	// All results must undo the initial projection
	switch (pnts.size())
	{
		case 0: return false;
		case 1:
		{
			TPoint2D p;
			project2D(pnts[0], pose, p);
			obj.data = p;
			return true;
		}
		case 2:
		{
			TSegment2D s;
			project2D(TSegment2D(pnts[0], pnts[1]), pose, s);
			obj.data = s;
			return true;
		}
		default: THROW_EXCEPTION("Polygon is not convex");
	}
}

// Auxiliary structs and code for 2D polygon intersection
struct T2ListsOfSegments
{
	vector<TSegment2D> l1;
	vector<TSegment2D> l2;
};

struct TCommonRegion
{
	std::shared_ptr<TPoint2D> point;
	std::shared_ptr<TSegment2D> segment;

	TCommonRegion(const TPoint2D& p) { point = std::make_shared<TPoint2D>(p); }
	TCommonRegion(const TSegment2D& s)
	{
		segment = std::make_shared<TSegment2D>(s);
	}
};

struct TTempIntersection
{
	std::shared_ptr<T2ListsOfSegments> segms;
	std::shared_ptr<TCommonRegion> common;

	TTempIntersection(const T2ListsOfSegments& s)
	{
		segms = std::make_shared<T2ListsOfSegments>(s);
	}
	TTempIntersection(const TCommonRegion& c)
	{
		common = std::make_shared<TCommonRegion>(c);
	}
};
struct TSegmentWithLine
{
	TSegment2D segment;
	TLine2D line;
	explicit TSegmentWithLine(const TSegment2D& s) : segment(s)
	{
		line = TLine2D(s[0], s[1]);
	}
	TSegmentWithLine(const TPoint2D& p1, const TPoint2D& p2) : segment(p1, p2)
	{
		line = TLine2D(p1, p2);
	}
	TSegmentWithLine() = default;
};
bool intersect(
	const TSegmentWithLine& s1, const TSegmentWithLine& s2, TObject2D& obj)
{
	if (!intersect(s1.line, s2.line, obj)) return false;
	if (obj.isLine())
		return intersectInCommonLine(s1.segment, s2.segment, s1.line, obj);
	TPoint2D p;
	obj.getPoint(p);
	return s1.segment.contains(p) && s2.segment.contains(p);
}
void getSegmentsWithLine(const TPolygon2D& poly, vector<TSegmentWithLine>& segs)
{
	size_t N = poly.size();
	segs.resize(N);
	for (size_t i = 0; i < N - 1; i++)
		segs[i] = TSegmentWithLine(poly[i], poly[i + 1]);
	segs[N - 1] = TSegmentWithLine(poly[N - 1], poly[0]);
}

bool math::intersect(
	const TPolygon2D& subject, const TPolygon2D& clipping, TObject2D& result)
{
	const auto p = intersect(subject, clipping);
	if (p.empty()) return false;

	result = TObject2D::From(p);
	return true;
}

double mrpt::math::signedArea(const mrpt::math::TPolygon2D& p)
{
	if (p.size() <= 2) return 0;
	double sa = 0;

	for (int i = 0; i < static_cast<int>(p.size()); i++)
	{
		const auto cp = p.at(i);
		const auto np = p.at((i + 1) % p.size());

		sa += cp.x * np.y - np.x * cp.y;
	}

	return sa * 0.5;
}

TPolygon2D mrpt::math::intersect(
	const TPolygon2D& subject, const TPolygon2D& clipping)
{
	/* Sutherland-Hodgman algorithm:
	 * https://en.wikipedia.org/wiki/Sutherland%E2%80%93Hodgman_algorithm
	 *
	 *List outputList = subjectPolygon;
	 *
	 *for (Edge clipEdge in clipPolygon) do
	 *    List inputList = outputList;
	 *    outputList.clear();
	 *
	 *    for (int i = 0; i < inputList.count; i += 1) do
	 *        Point current_point = inputList[i];
	 *        Point prev_point = inputList[(i − 1) % inputList.count];
	 *
	 *        Point Intersecting_point = ComputeIntersection(prev_point,
	 *current_point, clipEdge)
	 *
	 *        if (current_point inside clipEdge) then
	 *            if (prev_point not inside clipEdge) then
	 *                outputList.add(Intersecting_point);
	 *            end if
	 *            outputList.add(current_point);
	 *
	 *        else if (prev_point inside clipEdge) then
	 *            outputList.add(Intersecting_point);
	 *        end if
	 *
	 *    done
	 *done
	 */
	// We need polygons (may handle degenerated triangles=segments too)
	ASSERT_GE_(subject.size(), 2);
	ASSERT_GE_(clipping.size(), 2);

	TPolygon2D outputList = subject;

	std::vector<TSegmentWithLine> segsClip;
	getSegmentsWithLine(clipping, segsClip);

	const bool clipIsCW = signedArea(clipping) < 0;

	for (const auto& clipEdge : segsClip)
	{
		const auto inputList = outputList;
		outputList.clear();

		for (size_t i = 0; i < inputList.size(); i++)
		{
			const auto current_point = inputList.at(i);
			const auto prev_point =
				inputList.at(i > 0 ? (i - 1) : (inputList.size() - 1));

			const bool curInside =
				(clipEdge.line.signedDistance(current_point) < 0) ^ clipIsCW;
			const bool prevInside =
				(clipEdge.line.signedDistance(prev_point) < 0) ^ clipIsCW;

			mrpt::math::TObject2D intersectPt;
			const bool doIntersect = mrpt::math::intersect(
				clipEdge.line,
				mrpt::math::TLine2D::FromTwoPoints(current_point, prev_point),
				intersectPt);
			std::optional<mrpt::math::TPoint2D> iPt;
			if (doIntersect) { intersectPt.getPoint(iPt.emplace()); }

			if (curInside)
			{
				if (!prevInside)
				{
					if (iPt) outputList.push_back(*iPt);
				}
				outputList.push_back(current_point);
			}
			else if (prevInside)
			{
				if (iPt) outputList.push_back(*iPt);
			}
		}
	}

	return outputList;
}

bool math::intersect(const TPolygon3D& p1, const TSegment3D& s2, TObject3D& obj)
{
	TPlane p;
	if (!p1.getPlane(p)) return false;
	if (!intersect(p, s2, obj)) return false;
	TPoint3D pnt;
	TSegment3D sgm;
	if (obj.getPoint(pnt))
	{
		const TPose3D poseNeg = -p.getAsPose3DForcingOrigin(p1[0]);
		TPolygon3D projPoly;
		TPoint3D projPnt;
		project3D(p1, poseNeg, projPoly);
		project3D(pnt, poseNeg, projPnt);
		return TPolygon2D(projPoly).contains(TPoint2D(projPnt));
	}
	else if (obj.getSegment(sgm))
		return intersectInCommonPlane<TPolygon2D, TSegment2D>(p1, s2, p, obj);
	return false;
}

bool math::intersect(const TPolygon3D& p1, const TLine3D& r2, TObject3D& obj)
{
	TPlane p;
	if (!p1.getPlane(p)) return false;
	if (!intersect(p, r2, obj)) return false;
	TPoint3D pnt;
	if (obj.getPoint(pnt))
	{
		const TPose3D poseNeg = -p.getAsPose3DForcingOrigin(p1[0]);
		TPolygon3D projPoly;
		TPoint3D projPnt;
		project3D(p1, poseNeg, projPoly);
		project3D(pnt, poseNeg, projPnt);
		return TPolygon2D(projPoly).contains(TPoint2D(projPnt));
	}
	else if (obj.isLine())
		return intersectInCommonPlane<TPolygon2D, TLine2D>(p1, r2, p, obj);
	return false;
}

bool math::intersect(const TPolygon3D& p1, const TPlane& p2, TObject3D& obj)
{
	TPlane p;
	if (!p1.getPlane(p)) return false;
	if (!intersect(p, p2, obj)) return false;
	TLine3D ln;
	if (obj.isPlane())
	{
		// Polygon is inside the plane
		obj.data = p1;
		return true;
	}
	else if (obj.getLine(ln))
		return intersectInCommonPlane<TPolygon2D, TLine2D>(p1, ln, p, obj);
	return false;
}

// Auxiliary function2
bool intersectAux(
	const TPolygon3D& p1, const TPlane& pl1, const TPolygon3D& p2,
	const TPlane& pl2, TObject3D& obj)
{
	if (!intersect(pl1, pl2, obj)) return false;
	TLine3D ln;
	if (obj.isPlane())
		return intersectInCommonPlane<TPolygon2D, TPolygon2D>(
			p1, p2, pl1, obj);	// Polygons are on the same plane
	else if (obj.getLine(ln))
	{
		TObject3D obj1, obj2;
		if (!intersectInCommonPlane<TPolygon2D, TLine2D>(p1, ln, pl1, obj1))
			return false;
		if (!intersectInCommonPlane<TPolygon2D, TLine2D>(p2, ln, pl2, obj2))
			return false;
		TPoint3D po1, po2;
		TSegment3D s1, s2;
		if (obj1.getPoint(po1)) s1 = TSegment3D(po1, po1);
		else
			obj1.getSegment(s1);
		if (obj2.getPoint(po2)) s2 = TSegment3D(po2, po2);
		else
			obj2.getSegment(s2);
		return intersectInCommonLine(s1, s2, ln, obj);
	}
	return false;
}

bool compatibleBounds(
	const TPoint3D& min1, const TPoint3D& max1, const TPoint3D& min2,
	const TPoint3D& max2)
{
	for (size_t i = 0; i < 3; i++)
		if ((min1[i] > max2[i]) || (min2[i] > max1[i])) return false;
	return true;
}
// End of auxiliary functions

bool math::intersect(const TPolygon3D& p1, const TPolygon3D& p2, TObject3D& obj)
{
	TPoint3D min1, max1, min2, max2;
	getPrismBounds(p1, min1, max1);
	getPrismBounds(p2, min2, max2);
	if (!compatibleBounds(min1, max1, min2, max2)) return false;
	TPlane pl1, pl2;
	if (!p1.getPlane(pl1)) return false;
	if (!p2.getPlane(pl2)) return false;
	return intersectAux(p1, pl1, p2, pl2, obj);
}

// Auxiliary function
inline void getPlanes(
	const std::vector<TPolygon3D>& polys, std::vector<TPlane>& planes)
{
	size_t N = polys.size();
	planes.resize(N);
	for (size_t i = 0; i < N; i++)
		getRegressionPlane(polys[i], planes[i]);
}

// Auxiliary functions
void getMinAndMaxBounds(
	const std::vector<TPolygon3D>& v1, std::vector<TPoint3D>& minP,
	std::vector<TPoint3D>& maxP)
{
	minP.resize(0);
	maxP.resize(0);
	size_t N = v1.size();
	minP.reserve(N);
	maxP.reserve(N);
	TPoint3D p1, p2;
	for (const auto& it : v1)
	{
		getPrismBounds(it, p1, p2);
		minP.push_back(p1);
		maxP.push_back(p2);
	}
}

size_t math::intersect(
	const std::vector<TPolygon3D>& v1, const std::vector<TPolygon3D>& v2,
	CSparseMatrixTemplate<TObject3D>& objs)
{
	std::vector<TPlane> w1, w2;
	getPlanes(v1, w1);
	getPlanes(v2, w2);
	std::vector<TPoint3D> minBounds1, maxBounds1, minBounds2, maxBounds2;
	getMinAndMaxBounds(v1, minBounds1, maxBounds1);
	getMinAndMaxBounds(v2, minBounds2, maxBounds2);
	size_t M = v1.size(), N = v2.size();
	objs.clear();
	objs.resize(M, N);
	TObject3D obj;
	for (size_t i = 0; i < M; i++)
		for (size_t j = 0; j < N; j++)
			if (!compatibleBounds(
					minBounds1[i], maxBounds1[i], minBounds2[j], maxBounds2[j]))
				continue;
			else if (intersectAux(v1[i], w1[i], v2[j], w2[j], obj))
				objs(i, j) = obj;
	return objs.getNonNullElements();
}

size_t math::intersect(
	const std::vector<TPolygon3D>& v1, const std::vector<TPolygon3D>& v2,
	std::vector<TObject3D>& objs)
{
	objs.resize(0);
	std::vector<TPlane> w1, w2;
	getPlanes(v1, w1);
	getPlanes(v2, w2);
	std::vector<TPoint3D> minBounds1, maxBounds1, minBounds2, maxBounds2;
	getMinAndMaxBounds(v1, minBounds1, maxBounds1);
	getMinAndMaxBounds(v2, minBounds2, maxBounds2);
	TObject3D obj;
	auto itP1 = w1.begin();
	auto itMin1 = minBounds1.begin();
	auto itMax1 = maxBounds1.begin();
	for (auto it1 = v1.begin(); it1 != v1.end();
		 ++it1, ++itP1, ++itMin1, ++itMax1)
	{
		const TPolygon3D& poly1 = *it1;
		const TPlane& plane1 = *itP1;
		auto itP2 = w2.begin();
		const TPoint3D &min1 = *itMin1, max1 = *itMax1;
		auto itMin2 = minBounds2.begin();
		auto itMax2 = maxBounds2.begin();
		for (auto it2 = v2.begin(); it2 != v2.end();
			 ++it2, ++itP2, ++itMin2, ++itMax2)
			if (!compatibleBounds(min1, max1, *itMin2, *itMax2)) continue;
			else if (intersectAux(poly1, plane1, *it2, *itP2, obj))
				objs.push_back(obj);
	}
	return objs.size();
}

bool math::intersect(const TObject2D& o1, const TObject2D& o2, TObject2D& obj)
{
	TPoint2D p1, p2;
	TSegment2D s1, s2;
	TLine2D l1, l2;
	TPolygon2D po1, po2;
	if (o1.getPoint(p1))
	{
		obj.data = p1;
		if (o2.getPoint(p2)) return distance(p1, p2) < geometryEpsilon;
		else if (o2.getSegment(s2))
			return s2.contains(p1);
		else if (o2.getLine(l2))
			return l2.contains(p1);
		else if (o2.getPolygon(po2))
			return po2.contains(p1);  // else return false;
	}
	else if (o1.getSegment(s1))
	{
		if (o2.getPoint(p2))
		{
			if (s1.contains(p2))
			{
				obj.data = p2;
				return true;
			}  // else return false;
		}
		else if (o2.getSegment(s2))
			return intersect(s1, s2, obj);
		else if (o2.getLine(l2))
			return intersect(s1, l2, obj);
		else if (o2.getPolygon(po2))
			return intersect(s1, po2, obj);	 // else return false;
	}
	else if (o1.getLine(l1))
	{
		if (o2.getPoint(p2))
		{
			if (l1.contains(p2))
			{
				obj.data = p2;
				return true;
			}  // else return false;
		}
		else if (o2.getSegment(s2))
			return intersect(l1, s2, obj);
		else if (o2.getLine(l2))
			return intersect(l1, l2, obj);
		else if (o2.getPolygon(po2))
			return intersect(l1, po2, obj);	 // else return false;
	}
	else if (o1.getPolygon(po1))
	{
		if (o2.getPoint(p2))
		{
			if (po1.contains(p2))
			{
				obj.data = p2;
				return true;
			}  // else return false;
		}
		else if (o2.getSegment(s2))
			return intersect(po1, s2, obj);
		else if (o2.getLine(l2))
			return intersect(po1, l2, obj);
		else if (o2.getPolygon(po2))
			return intersect(po1, po2, obj);  // else return false;
	}  // else return false;
	return false;
}

bool math::intersect(const TObject3D& o1, const TObject3D& o2, TObject3D& obj)
{
	TPoint3D p1, p2;
	TSegment3D s1, s2;
	TLine3D l1, l2;
	TPolygon3D po1, po2;
	TPlane pl1, pl2;
	if (o1.getPoint(p1))
	{
		obj.data = p1;
		if (o2.getPoint(p2)) return distance(p1, p2) < geometryEpsilon;
		else if (o2.getSegment(s2))
			return s2.contains(p1);
		else if (o2.getLine(l2))
			return l2.contains(p1);
		else if (o2.getPolygon(po2))
			return po2.contains(p1);
		else if (o2.getPlane(pl2))
			return pl2.contains(p1);  // else return false;
	}
	else if (o1.getSegment(s1))
	{
		if (o2.getPoint(p2))
		{
			if (s1.contains(p2))
			{
				obj.data = p2;
				return true;
			}  // else return false;
		}
		else if (o2.getSegment(s2))
			return intersect(s1, s2, obj);
		else if (o2.getLine(l2))
			return intersect(s1, l2, obj);
		else if (o2.getPolygon(po2))
			return intersect(s1, po2, obj);
		else if (o2.getPlane(pl2))
			return intersect(s1, pl2, obj);	 // else return false;
	}
	else if (o1.getLine(l1))
	{
		if (o2.getPoint(p2))
		{
			if (l1.contains(p2))
			{
				obj.data = p2;
				return true;
			}  // else return false;
		}
		else if (o2.getSegment(s2))
			return intersect(l1, s2, obj);
		else if (o2.getLine(l2))
			return intersect(l1, l2, obj);
		else if (o2.getPolygon(po2))
			return intersect(l1, po2, obj);
		else if (o2.getPlane(pl2))
			return intersect(l2, pl2, obj);	 // else return false;
	}
	else if (o1.getPolygon(po1))
	{
		if (o2.getPoint(p2))
		{
			if (po1.contains(p2))
			{
				obj.data = p2;
				return true;
			}  // else return false;
		}
		else if (o2.getSegment(s2))
			return intersect(po1, s2, obj);
		else if (o2.getLine(l2))
			return intersect(po1, l2, obj);
		else if (o2.getPolygon(po2))
			return intersect(po1, po2, obj);
		else if (o2.getPlane(pl2))
			return intersect(po1, pl2, obj);  // else return false;
	}
	else if (o1.getPlane(pl1))
	{
		if (o2.getPoint(p2))
		{
			if (pl1.contains(p2))
			{
				obj.data = p2;
				return true;
			}  // else return false;
		}
		else if (o2.getSegment(s2))
			return intersect(pl1, s2, obj);
		else if (o2.getLine(l2))
			return intersect(pl1, l2, obj);
		else if (o2.getPlane(pl2))
			return intersect(pl1, pl2, obj);  // else return false;
	}  // else return false;
	return false;
}

double math::distance(const TPoint2D& p1, const TPoint2D& p2)
{
	double dx = p2.x - p1.x;
	double dy = p2.y - p1.y;
	return sqrt(dx * dx + dy * dy);
}

double math::distance(const TPoint3D& p1, const TPoint3D& p2)
{
	double dx = p2.x - p1.x;
	double dy = p2.y - p1.y;
	double dz = p2.z - p1.z;
	return sqrt(dx * dx + dy * dy + dz * dz);
}

void math::getRectangleBounds(
	const std::vector<TPoint2D>& poly, TPoint2D& pMin, TPoint2D& pMax)
{
	size_t N = poly.size();
	if (N < 1) throw logic_error("Empty polygon");
	pMin = poly[0];
	pMax = poly[0];
	for (size_t i = 1; i < N; i++)
	{
		pMin.x = min(pMin.x, poly[i].x);
		pMin.y = min(pMin.y, poly[i].y);
		pMax.x = max(pMax.x, poly[i].x);
		pMax.y = max(pMax.y, poly[i].y);
	}
}

double math::distance(const TLine2D& r1, const TLine2D& r2)
{
	if (std::abs(r1.coefs[0] * r2.coefs[1] - r2.coefs[0] * r1.coefs[1]) <
		geometryEpsilon)
	{
		// Lines are parallel
		size_t i1 = (std::abs(r1.coefs[0]) < geometryEpsilon) ? 0 : 1;
		TPoint2D p;
		p[i1] = 0.0;
		p[1 - i1] = -r1.coefs[2] / r1.coefs[1 - i1];
		return r2.distance(p);
	}
	else
		return 0;  // Lines cross in some point
}

double math::distance(const TLine3D& r1, const TLine3D& r2)
{
	if (std::abs(getAngle(r1, r2)) < geometryEpsilon)
		return r1.distance(r2.pBase);  // Lines are parallel
	else
	{
		// We build a plane parallel to r2 which contains r1
		TPlane p;
		crossProduct3D(r1.director, r2.director, p.coefs);
		p.coefs[3] =
			-(p.coefs[0] * r1.pBase[0] + p.coefs[1] * r1.pBase[1] +
			  p.coefs[2] * r1.pBase[2]);
		return p.distance(r2.pBase);
	}
}

double math::distance(const TPlane& p1, const TPlane& p2)
{
	if (std::abs(getAngle(p1, p2)) < geometryEpsilon)
	{
		// Planes are parallel
		TPoint3D p(0, 0, 0);
		for (size_t i = 0; i < 3; i++)
			if (std::abs(p1.coefs[i]) >= geometryEpsilon)
			{
				p[i] = -p1.coefs[3] / p1.coefs[i];
				break;
			}
		return p2.distance(p);
	}
	else
		return 0;  // Planes cross in a line
}

double math::distance(
	[[maybe_unused]] const TPolygon2D& p1,
	[[maybe_unused]] const TPolygon2D& p2)
{
	THROW_EXCEPTION("TO DO:distance(TPolygon2D,TPolygon2D)");
}

double math::distance(
	[[maybe_unused]] const TPolygon2D& p1,
	[[maybe_unused]] const TSegment2D& s2)
{
	THROW_EXCEPTION("TO DO:distance(TPolygon2D,TSegment)");
}

double math::distance(
	[[maybe_unused]] const TPolygon2D& p1, [[maybe_unused]] const TLine2D& l2)
{
	THROW_EXCEPTION("TO DO:distance(TPolygon2D,TLine2D)");
}

double math::distance(
	[[maybe_unused]] const TPolygon3D& p1,
	[[maybe_unused]] const TPolygon3D& p2)
{
	THROW_EXCEPTION("TO DO:distance(TPolygon3D,TPolygon3D");
}

double math::distance(
	[[maybe_unused]] const TPolygon3D& p1,
	[[maybe_unused]] const TSegment3D& s2)
{
	THROW_EXCEPTION("TO DO:distance(TPolygon3D,TSegment3D");
}

double math::distance(
	[[maybe_unused]] const TPolygon3D& p1, [[maybe_unused]] const TLine3D& l2)
{
	THROW_EXCEPTION("TO DO:distance(TPolygon3D,TLine3D");
}

double math::distance(
	[[maybe_unused]] const TPolygon3D& po, [[maybe_unused]] const TPlane& pl)
{
	THROW_EXCEPTION("TO DO:distance(TPolygon3D,TPlane");
}

void math::getPrismBounds(
	const std::vector<TPoint3D>& poly, TPoint3D& pMin, TPoint3D& pMax)
{
	size_t N = poly.size();
	if (N < 1) throw logic_error("Empty polygon");
	pMin = poly[0];
	pMax = poly[0];
	for (size_t i = 1; i < N; i++)
	{
		pMin.x = min(pMin.x, poly[i].x);
		pMin.y = min(pMin.y, poly[i].y);
		pMin.z = min(pMin.z, poly[i].z);
		pMax.x = max(pMax.x, poly[i].x);
		pMax.y = max(pMax.y, poly[i].y);
		pMax.z = max(pMax.z, poly[i].z);
	}
}

void createPlaneFromPoseAndAxis(const TPose3D& pose, TPlane& plane, size_t axis)
{
	plane.coefs[3] = 0;
	CMatrixDouble44 m;
	pose.getHomogeneousMatrix(m);
	for (size_t i = 0; i < 3; i++)
	{
		plane.coefs[i] = m(i, axis);
		plane.coefs[3] -= plane.coefs[i] * m(i, 3);
	}
}

void math::createPlaneFromPoseXY(const TPose3D& pose, TPlane& plane)
{
	createPlaneFromPoseAndAxis(pose, plane, 2);
}

void math::createPlaneFromPoseXZ(const TPose3D& pose, TPlane& plane)
{
	createPlaneFromPoseAndAxis(pose, plane, 1);
}

void math::createPlaneFromPoseYZ(const TPose3D& pose, TPlane& plane)
{
	createPlaneFromPoseAndAxis(pose, plane, 0);
}

void math::createPlaneFromPoseAndNormal(
	const TPose3D& pose, const double (&normal)[3], TPlane& plane)
{
	plane.coefs[3] = 0;
	CMatrixDouble44 m;
	pose.getHomogeneousMatrix(m);
	for (size_t i = 0; i < 3; i++)
	{
		plane.coefs[i] = 0;
		for (size_t j = 0; j < 3; j++)
			plane.coefs[i] += normal[j] * m(i, j);
		plane.coefs[3] -= plane.coefs[i] * m(i, 3);
	}
}

CMatrixDouble44 math::generateAxisBaseFromDirectionAndAxis(
	const TVector3D& vec, uint8_t coord)
{
	CMatrixDouble44 m;
	// Assumes vector is unitary.
	// coord: 0=x, 1=y, 2=z.
	const uint8_t coord1 = (coord + 1) % 3;
	const uint8_t coord2 = (coord + 2) % 3;
	m.setZero();
	m(3, 3) = 1.0;
	for (size_t i = 0; i < 3; i++)
		m(i, coord) = vec[i];
	m(0, coord1) = 0;
	double h = hypot(vec[1], vec[2]);
	if (h < geometryEpsilon)
	{
		m(1, coord1) = 1;
		m(2, coord1) = 0;
	}
	else
	{
		m(1, coord1) = -vec[2] / h;
		m(2, coord1) = vec[1] / h;
	}
	m(0, coord2) = m(1, coord) * m(2, coord1) - m(2, coord) * m(1, coord1);
	m(1, coord2) = m(2, coord) * m(0, coord1) - m(0, coord) * m(2, coord1);
	m(2, coord2) = m(0, coord) * m(1, coord1) - m(1, coord) * m(0, coord1);
	return m;
}

double math::getRegressionLine(const vector<TPoint2D>& points, TLine2D& line)
{
	CVectorFixedDouble<2> means;
	CMatrixDouble22 covars;
	covariancesAndMean(points, covars, means);

	std::vector<double> eigenVals;
	CMatrixDouble22 eigenVecs;
	covars.eig_symmetric(eigenVecs, eigenVals);

	// size_t selected = (eigenVal[0] >= eigenVal[1]) ? 0 : 1;
	line.coefs[0] = -eigenVecs(1, 1);  // 1: largest eigenVal
	line.coefs[1] = eigenVecs(0, 1);
	line.coefs[2] = -line.coefs[0] * means[0] - line.coefs[1] * means[1];
	return std::sqrt(eigenVals[0] / eigenVals[1]);
}

template <class T>
inline size_t getIndexOfMin(const T& e1, const T& e2, const T& e3)
{
	return (e1 < e2) ? ((e1 < e3) ? 0 : 2) : ((e2 < e3) ? 1 : 2);
}

double math::getRegressionLine(const vector<TPoint3D>& points, TLine3D& line)
{
	CVectorFixedDouble<3> means;
	CMatrixDouble33 covars;
	covariancesAndMean(points, covars, means);

	std::vector<double> eigenVal;
	CMatrixDouble33 eigenVec;
	covars.eig_symmetric(eigenVec, eigenVal);

	const size_t selected = 2;	// sorted: max eigenvalue
	for (size_t i = 0; i < 3; i++)
	{
		line.pBase[i] = means[i];
		line.director[i] = eigenVec(i, selected);
	}
	size_t i1 = (selected + 1) % 3, i2 = (selected + 2) % 3;
	return std::sqrt((eigenVal[i1] + eigenVal[i2]) / eigenVal[selected]);
}

double math::getRegressionPlane(const vector<TPoint3D>& points, TPlane& plane)
{
	vector<double> means;
	CMatrixDouble33 covars;
	covariancesAndMean(points, covars, means);

	std::vector<double> eigenVal;
	CMatrixDouble33 eigenVec;
	covars.eig_symmetric(eigenVec, eigenVal);

	for (size_t i = 0; i < 3; ++i)
	{
		if (eigenVal[i] < 0 && std::abs(eigenVal[i]) < geometryEpsilon)
			eigenVal[i] = 0;
	}

	if (std::abs(eigenVal.at(1)) < geometryEpsilon)
		THROW_EXCEPTION(
			"Points are aligned, cannot fit a plane (infinite solutions)");

	const size_t selected = 0;	// sorted: minimum eigenVal
	plane.coefs[3] = 0;
	for (size_t i = 0; i < 3; i++)
	{
		plane.coefs[i] = eigenVec(i, selected);
		plane.coefs[3] -= plane.coefs[i] * means[i];
	}
	size_t i1 = (selected + 1) % 3, i2 = (selected + 2) % 3;
	const auto condNumbInv = eigenVal[selected] / (eigenVal[i1] + eigenVal[i2]);
	return condNumbInv;
}

void math::assemblePolygons(
	const std::vector<TSegment3D>& segms, std::vector<TPolygon3D>& polys)
{
	std::vector<TSegment3D> tmp;
	assemblePolygons(segms, polys, tmp);
}

struct MatchingVertex
{
	size_t seg1;
	size_t seg2;
	bool seg1Point;	 // true for point2, false for point1
	bool seg2Point;	 // same
	MatchingVertex() = default;
	MatchingVertex(size_t s1, size_t s2, bool s1p, bool s2p)
		: seg1(s1), seg2(s2), seg1Point(s1p), seg2Point(s2p)
	{
	}
};
class FCreatePolygon
{
   public:
	const std::vector<TSegment3D>& segs;
	FCreatePolygon(const std::vector<TSegment3D>& s) : segs(s) {}
	TPolygon3D operator()(const std::vector<MatchingVertex>& vertices)
	{
		TPolygon3D res;
		size_t N = vertices.size();
		res.reserve(N);
		for (const auto& vertice : vertices)
			res.push_back(segs[vertice.seg2][vertice.seg2Point ? 1 : 0]);
		return res;
	}
};
inline bool firstOrNonPresent(size_t i, const std::vector<MatchingVertex>& v)
{
	if (v.size() > 0 && v[0].seg1 == i) return true;
	for (const auto& it : v)
		if (it.seg1 == i || it.seg2 == i) return false;
	return true;
}
bool depthFirstSearch(
	const CSparseMatrixTemplate<unsigned char>& mat,
	std::vector<std::vector<MatchingVertex>>& res, std::vector<bool>& used,
	size_t searching, unsigned char mask, std::vector<MatchingVertex>& current)
{
	for (size_t i = 0; i < mat.cols(); i++)
		if (!used[i] && mat.isNotNull(searching, i))
		{
			unsigned char match = mat(searching, i) & mask;
			if (!match) continue;
			else if (firstOrNonPresent(i, current))
			{
				bool s1p, s2p;
				if (true == (s1p = (!(match & 3)))) match >>= 2;
				s2p = !(match & 1);
				if (current.size() >= 2 && current[0].seg1 == i)
				{
					if (s2p != current[0].seg1Point)
					{
						current.emplace_back(searching, i, s1p, s2p);
						for (auto it = current.begin(); it != current.end();
							 ++it)
							used[it->seg2] = true;
						res.push_back(current);
						return true;
					}
					else
						continue;  // Strange shape... not a polygon, although
					// it'll be without the first segment
				}
				else
				{
					current.emplace_back(searching, i, s1p, s2p);
					if (depthFirstSearch(
							mat, res, used, i, s2p ? 0x3 : 0xC, current))
						return true;
					current.pop_back();
				}
			}
		}
	// No match has been found. Backtrack
	return false;
}
void depthFirstSearch(
	const CSparseMatrixTemplate<unsigned char>& mat,
	std::vector<std::vector<MatchingVertex>>& res, std::vector<bool>& used)
{
	vector<MatchingVertex> cur;
	for (size_t i = 0; i < used.size(); i++)
		if (!used[i])
			if (depthFirstSearch(mat, res, used, i, 0xf, cur)) cur.clear();
}
void math::assemblePolygons(
	const std::vector<TSegment3D>& segms, std::vector<TPolygon3D>& polys,
	std::vector<TSegment3D>& remainder)
{
	std::vector<TSegment3D> tmp;
	tmp.reserve(segms.size());
	for (const auto& segm : segms)
		if (segm.length() >= geometryEpsilon) tmp.push_back(segm);
		else
			remainder.push_back(segm);
	size_t N = tmp.size();
	CSparseMatrixTemplate<unsigned char> matches(N, N);
	for (size_t i = 0; i < N - 1; i++)
		for (size_t j = i + 1; j < N; j++)
		{
			if (distance(tmp[i].point1, tmp[j].point1) < geometryEpsilon)
			{
				matches(i, j) |= 1;
				matches(j, i) |= 1;
			}
			if (distance(tmp[i].point1, tmp[j].point2) < geometryEpsilon)
			{
				matches(i, j) |= 2;
				matches(j, i) |= 4;
			}
			if (distance(tmp[i].point2, tmp[j].point1) < geometryEpsilon)
			{
				matches(i, j) |= 4;
				matches(j, i) |= 2;
			}
			if (distance(tmp[i].point2, tmp[j].point2) < geometryEpsilon)
			{
				matches(i, j) |= 8;
				matches(j, i) |= 8;
			}
		}
	std::vector<std::vector<MatchingVertex>> results;
	std::vector<bool> usedSegments(N, false);
	depthFirstSearch(matches, results, usedSegments);
	polys.resize(results.size());
	transform(
		results.begin(), results.end(), polys.begin(), FCreatePolygon(segms));
	for (size_t i = 0; i < N; i++)
		if (!usedSegments[i]) remainder.push_back(tmp[i]);
}

namespace
{
std::vector<TPolygon3D> getPolygons(
	const std::vector<TObject3D>& objs,
	const mrpt::optional_ref<std::vector<mrpt::math::TObject3D>>& others =
		std::nullopt)
{
	std::vector<TPolygon3D> r;
	for (const auto& o : objs)
	{
		if (o.isPolygon()) r.emplace_back(o.getAs<TPolygon3D>());
		else if (others)
			others->get().push_back(o);
	}
	return r;
}
std::vector<TSegment3D> getSegments(
	const std::vector<TObject3D>& objs,
	const mrpt::optional_ref<std::vector<mrpt::math::TObject3D>>& others =
		std::nullopt)

{
	std::vector<TSegment3D> r;
	for (const auto& o : objs)
	{
		if (o.isSegment()) r.emplace_back(o.getAs<TSegment3D>());
		else if (others)
			others->get().push_back(o);
	}
	return r;
}
}  // namespace

void math::assemblePolygons(
	const std::vector<TObject3D>& objs, std::vector<TPolygon3D>& polys)
{
	polys = getPolygons(objs);
	const auto segms = getSegments(objs);
	assemblePolygons(segms, polys);
}

void math::assemblePolygons(
	const std::vector<TObject3D>& objs, std::vector<TPolygon3D>& polys,
	std::vector<TObject3D>& remainder)
{
	std::vector<TObject3D> tmp;
	polys = getPolygons(objs, tmp);

	std::vector<TSegment3D> remainderSgms;
	const auto sgms = getSegments(tmp, remainder);
	assemblePolygons(sgms, polys, remainderSgms);
	for (const auto& o : remainderSgms)
		remainder.emplace_back(TObject3D::From(o));
}

void math::assemblePolygons(
	const std::vector<TObject3D>& objs, std::vector<TPolygon3D>& polys,
	std::vector<TSegment3D>& remainder1, std::vector<TObject3D>& remainder2)
{
	std::vector<TObject3D> tmp;
	polys = getPolygons(objs, tmp);
	std::vector<TSegment3D> sgms = getSegments(tmp, remainder2);
	assemblePolygons(sgms, polys, remainder1);
}

bool intersect(const TLine2D& l1, const TSegmentWithLine& s2, TObject2D& obj)
{
	if (intersect(l1, s2.line, obj))
	{
		if (obj.isLine())
		{
			obj.data = s2.segment;
			return true;
		}
		else
		{
			TPoint2D p;
			obj.getPoint(p);
			return s2.segment.contains(p);
		}
	}
	else
		return false;
}

inline bool intersect(
	const TSegmentWithLine& s1, const TLine2D& l2, TObject2D& obj)
{
	return intersect(l2, s1, obj);
}

bool math::splitInConvexComponents(
	const TPolygon2D& poly, vector<TPolygon2D>& components)
{
	components.clear();
	size_t N = poly.size();
	if (N <= 3) return false;
	vector<TSegmentWithLine> segms(N);
	for (size_t i = 0; i < N - 1; i++)
		segms[i] = TSegmentWithLine(poly[i], poly[i + 1]);
	segms[N - 1] = TSegmentWithLine(poly[N - 1], poly[0]);
	TObject2D obj;
	TPoint2D pnt;
	for (size_t i = 0; i < N; i++)
	{
		size_t ii = (i + 2) % N, i_ = (i + N - 1) % N;
		for (size_t j = ii; j != i_; j = (j + 1) % N)
			if (intersect(segms[i].line, segms[j], obj) && obj.getPoint(pnt))
			{
				TSegmentWithLine sTmp = TSegmentWithLine(
					pnt,
					segms[i].segment
						[(distance(pnt, segms[i].segment.point1) <
						  distance(pnt, segms[i].segment.point2))
							 ? 0
							 : 1]);
				bool cross = false;
				TPoint2D pTmp;
				for (size_t k = 0; (k < N) && !cross; k++)
					if (intersect(sTmp, segms[k], obj))
					{
						if (obj.getPoint(pTmp) &&
							(distance(pTmp, sTmp.segment[0]) >=
							 geometryEpsilon) &&
							(distance(pTmp, sTmp.segment[1]) >=
							 geometryEpsilon))
							cross = true;
					}
				if (cross) continue;
				// At this point, we have a suitable point, although we must
				// check if the division is right.
				// We do this by evaluating, in the expanded segment's line, the
				// next and previous points. If both signs differ, proceed.
				if (sign(segms[i].line.evaluatePoint(poly[(i + N - 1) % N])) ==
					sign(segms[i].line.evaluatePoint(poly[(i + 2) % N])))
					continue;
				TPolygon2D p1, p2;
				if (i > j)
				{
					p1.insert(p1.end(), poly.begin() + i + 1, poly.end());
					p1.insert(p1.end(), poly.begin(), poly.begin() + j + 1);
					p2.insert(
						p2.end(), poly.begin() + j + 1, poly.begin() + i + 1);
				}
				else
				{
					p1.insert(
						p1.end(), poly.begin() + i + 1, poly.begin() + j + 1);
					p2.insert(p2.end(), poly.begin() + j + 1, poly.end());
					p2.insert(p2.end(), poly.begin(), poly.begin() + i + 1);
				}
				if (distance(*(p1.rbegin()), pnt) >= geometryEpsilon)
					p1.push_back(pnt);
				if (distance(*(p2.rbegin()), pnt) >= geometryEpsilon)
					p2.push_back(pnt);
				p1.removeRedundantVertices();
				p2.removeRedundantVertices();
				vector<TPolygon2D> tempComps;
				if (splitInConvexComponents(p1, tempComps))
					components.insert(
						components.end(), tempComps.begin(), tempComps.end());
				else
					components.push_back(p1);
				if (splitInConvexComponents(p2, tempComps))
					components.insert(
						components.end(), tempComps.begin(), tempComps.end());
				else
					components.push_back(p2);
				return true;
			}
	}
	return false;
}

class FUnprojectPolygon2D
{
   public:
	const TPose3D& pose;
	TPolygon3D tmp1, tmp2;
	FUnprojectPolygon2D(const TPose3D& p) : pose(p), tmp1(0), tmp2(0) {}
	TPolygon3D operator()(const TPolygon2D& poly2D)
	{
		tmp1 = TPolygon3D(poly2D);
		project3D(tmp1, pose, tmp2);
		return tmp2;
	}
};
bool math::splitInConvexComponents(
	const TPolygon3D& poly, vector<TPolygon3D>& components)
{
	TPlane p;
	if (!poly.getPlane(p)) THROW_EXCEPTION("Polygon is skew");
	TPose3D pose1;
	p.getAsPose3DForcingOrigin(poly[0], pose1);
	const TPose3D pose2 = -pose1;
	TPolygon3D polyTmp;
	project3D(poly, pose2, polyTmp);
	TPolygon2D poly2D = TPolygon2D(polyTmp);
	vector<TPolygon2D> components2D;
	if (splitInConvexComponents(poly2D, components2D))
	{
		components.resize(components2D.size());
		transform(
			components2D.begin(), components2D.end(), components.begin(),
			FUnprojectPolygon2D(pose1));
		return true;
	}
	else
		return false;
}

void math::getSegmentBisector(const TSegment2D& sgm, TLine2D& bis)
{
	TPoint2D p;
	sgm.getCenter(p);
	bis.coefs[0] = sgm.point2.x - sgm.point1.x;
	bis.coefs[1] = sgm.point2.y - sgm.point1.y;
	bis.coefs[2] = -bis.coefs[0] * p.x - bis.coefs[1] * p.y;
	bis.unitarize();
}

void math::getSegmentBisector(const TSegment3D& sgm, TPlane& bis)
{
	TPoint3D p;
	sgm.getCenter(p);
	bis.coefs[0] = sgm.point2.x - sgm.point1.x;
	bis.coefs[1] = sgm.point2.y - sgm.point1.y;
	bis.coefs[2] = sgm.point2.z - sgm.point1.z;
	bis.coefs[2] =
		-bis.coefs[0] * p.x - bis.coefs[1] * p.y - bis.coefs[2] * p.z;
	bis.unitarize();
}

void math::getAngleBisector(const TLine2D& l1, const TLine2D& l2, TLine2D& bis)
{
	TPoint2D p;
	TObject2D obj;
	if (!intersect(l1, l2, obj))
	{
		// Both lines are parallel
		double mod1 = sqrt(square(l1.coefs[0]) + square(l1.coefs[1]));
		double mod2 = sqrt(square(l2.coefs[0]) + square(l2.coefs[2]));
		bis.coefs[0] = l1.coefs[0] / mod1;
		bis.coefs[1] = l1.coefs[1] / mod1;
		bool sameSign;
		if (std::abs(bis.coefs[0]) < geometryEpsilon)
			sameSign = (l1.coefs[1] * l2.coefs[1]) > 0;
		else
			sameSign = (l1.coefs[0] * l2.coefs[0]) > 0;
		if (sameSign)
			bis.coefs[2] = (l1.coefs[2] / mod1) + (l2.coefs[2] / mod2);
		else
			bis.coefs[2] = (l1.coefs[2] / mod1) - (l2.coefs[2] / mod2);
	}
	else if (obj.getPoint(p))
	{
		// Both lines cross
		double ang1 = atan2(-l1.coefs[0], l1.coefs[1]);
		double ang2 = atan2(-l2.coefs[0], l2.coefs[1]);
		double ang = (ang1 + ang2) / 2;
		bis.coefs[0] = -sin(ang);
		bis.coefs[1] = cos(ang);
		bis.coefs[2] = -bis.coefs[0] * p.x - bis.coefs[1] * p.y;
	}
	else
	{
		bis = l1;
		bis.unitarize();
	}
}

void math::getAngleBisector(const TLine3D& l1, const TLine3D& l2, TLine3D& bis)
{
	TPlane p = TPlane(l1, l2);	// May throw an exception
	TLine3D l1P, l2P;
	TLine2D bis2D;
	TPose3D pose, pose2;
	p.getAsPose3D(pose);
	pose2 = -pose;
	project3D(l1, pose2, l1P);
	project3D(l2, pose2, l2P);
	getAngleBisector(TLine2D(l1P), TLine2D(l2P), bis2D);
	project3D(TLine3D(bis2D), pose, bis);
}

bool math::traceRay(
	const vector<TPolygonWithPlane>& vec, const TPose3D& pose, double& dist)
{
	dist = HUGE_VAL;
	double nDist = 0;
	TLine3D lin;
	createFromPoseX(pose, lin);
	lin.unitarize();
	bool res = false;
	for (const auto& it : vec)
		if (::intersect(it, lin, nDist, dist))
		{
			res = true;
			dist = nDist;
		}
	return res;
}

CMatrixDouble33 mrpt::math::generateAxisBaseFromDirection(
	double dx, double dy, double dz)
{
	MRPT_START

	if (std::abs(dx) < 1e-10 && std::abs(dy) < 1e-10 && std::abs(dz) < 1e-10)
		THROW_EXCEPTION("Invalid input: Direction vector is (0,0,0);");

	CMatrixDouble33 P;

	// 1st vector:
	double n_xy = square(dx) + square(dy);
	double n = sqrt(n_xy + square(dz));
	n_xy = sqrt(n_xy);
	P(0, 0) = dx / n;
	P(1, 0) = dy / n;
	P(2, 0) = dz / n;

	// 2nd perpendicular vector:
	if (fabs(dx) > 1e-4 || fabs(dy) > 1e-4)
	{
		P(0, 1) = -dy / n_xy;
		P(1, 1) = dx / n_xy;
		P(2, 1) = 0;
	}
	else
	{
		// Any vector in the XY plane will work:
		P(0, 1) = 1;
		P(1, 1) = 0;
		P(2, 1) = 0;
	}

	// 3rd perpendicular vector: cross product of the two last vectors:
	Eigen::Vector3d c2;
	crossProduct3D(P.col(0), P.col(1), c2);
	P.col(2) = c2;

	return P;
	MRPT_END
}
