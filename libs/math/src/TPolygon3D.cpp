/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "math-precomp.h"  // Precompiled headers
//
#include <mrpt/containers/yaml.h>
#include <mrpt/math/TLine2D.h>
#include <mrpt/math/TPlane.h>
#include <mrpt/math/TPolygon2D.h>
#include <mrpt/math/TPolygon3D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TSegment2D.h>
#include <mrpt/math/epsilon.h>
#include <mrpt/math/geometry.h>	 // project3D()

#include <iostream>

#include "polygons_utils.h"

using namespace mrpt::math;

void TPolygon3D::generate2DObject(TPolygon2D& p) const
{
	p = TPolygon2D(*this);
}

double TPolygon3D::distance(const TPoint3D& point) const
{
	TPlane pl;
	if (!getPlane(pl))
		throw std::logic_error("Polygon does not conform a plane");
	TPoint3D newPoint;
	TPolygon3D newPoly;
	TPose3D pose;
	pl.getAsPose3DForcingOrigin(operator[](0), pose);
	project3D(point, pose, newPoint);
	project3D(*this, pose, newPoly);
	double distance2D = TPolygon2D(newPoly).distance(TPoint2D(newPoint));
	return sqrt(newPoint.z * newPoint.z + distance2D * distance2D);
}
bool TPolygon3D::contains(const TPoint3D& point) const
{
	TPoint3D pMin, pMax;
	getPrismBounds(*this, pMin, pMax);
	if (point.x + getEpsilon() < pMin.x || point.y + getEpsilon() < pMin.y ||
		point.z + getEpsilon() < pMin.z || point.x > pMax.x + getEpsilon() ||
		point.y > pMax.y + getEpsilon() || point.z > pMax.z + getEpsilon())
		return false;
	TPlane plane;
	if (!getPlane(plane))
		throw std::logic_error("Polygon does not conform a plane");
	TPolygon3D projectedPoly;
	TPoint3D projectedPoint;
	TPose3D pose;
	// plane.getAsPose3DForcingOrigin(operator[](0),pose);
	plane.getAsPose3D(pose);
	CMatrixDouble44 P_inv;
	pose.getInverseHomogeneousMatrix(P_inv);
	pose.fromHomogeneousMatrix(P_inv);
	project3D(point, pose, projectedPoint);
	if (std::abs(projectedPoint.z) >= getEpsilon())
		return false;  // Point is not inside the polygon's plane.
	project3D(*this, pose, projectedPoly);
	return TPolygon2D(projectedPoly).contains(TPoint2D(projectedPoint));
}
void TPolygon3D::getAsSegmentList(vector<TSegment3D>& v) const
{
	size_t N = size();
	v.resize(N);
	for (size_t i = 0; i < N - 1; i++)
		v[i] = TSegment3D(operator[](i), operator[](i + 1));
	v[N - 1] = TSegment3D(operator[](N - 1), operator[](0));
}
bool TPolygon3D::getPlane(TPlane& p) const { return conformAPlane(*this, p); }
void TPolygon3D::getBestFittingPlane(TPlane& p) const
{
	getRegressionPlane(*this, p);
}
void TPolygon3D::getCenter(TPoint3D& p) const
{
	std::for_each(begin(), end(), FAddPoint<TPoint3D, 3>(p));
	size_t N = size();
	p.x /= N;
	p.y /= N;
	p.z /= N;
}
bool TPolygon3D::isSkew() const { return !mrpt::math::conformAPlane(*this); }
void TPolygon3D::removeRepeatedVertices() { removeRepVertices(*this); }
void TPolygon3D::removeRedundantVertices()
{
	removeRepeatedVertices();
	removeUnusedVertices(*this);
}
TPolygon3D::TPolygon3D(const TPolygon2D& p) : std::vector<TPoint3D>()
{
	size_t N = p.size();
	resize(N);
	for (size_t i = 0; i < N; i++)
		operator[](i) = p[i];
}
void TPolygon3D::createRegularPolygon(
	size_t numEdges, double radius, TPolygon3D& poly)
{
	if (numEdges < 3 || std::abs(radius) < getEpsilon())
		throw std::logic_error(
			"Invalid arguments for regular polygon creations");
	poly.resize(numEdges);
	for (size_t i = 0; i < numEdges; i++)
	{
		double angle = i * 2 * M_PI / numEdges;
		poly[i] = TPoint3D(radius * cos(angle), radius * sin(angle), 0);
	}
}
void TPolygon3D::createRegularPolygon(
	size_t numEdges, double radius, TPolygon3D& poly, const TPose3D& pose)
{
	createRegularPolygon(numEdges, radius, poly);
	for (size_t i = 0; i < numEdges; i++)
		pose.composePoint(poly[i], poly[i]);
}

std::ostream& mrpt::math::operator<<(std::ostream& o, const TPolygon3D& p)
{
	o << "mrpt::math::TPolygon3D vertices:\n";
	for (const auto& v : p)
		o << " - " << v << "\n";
	return o;
}

TPolygon3D TPolygon3D::FromYAML(const mrpt::containers::yaml& c)
{
	if (c.isNullNode() || c.empty()) return {};
	TPolygon3D p;
	ASSERT_(c.isSequence());
	for (const auto& vertex : c.asSequence())
	{
		ASSERT_(vertex.isSequence());
		const auto& vertexData = vertex.asSequence();
		ASSERT_EQUAL_(vertexData.size(), 3U);
		p.emplace_back(
			vertexData.at(0).as<double>(), vertexData.at(1).as<double>(),
			vertexData.at(2).as<double>());
	}

	return p;
}

mrpt::containers::yaml TPolygon3D::asYAML() const
{
	mrpt::containers::yaml c = mrpt::containers::yaml::Sequence();

	for (const auto& vertex : *this)
	{
		auto pts =
			mrpt::containers::yaml::Sequence({vertex.x, vertex.y, vertex.z});
		pts.printInShortFormat = true;
		c.push_back(pts);
	}

	return c;
}
