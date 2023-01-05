/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "math-precomp.h"  // Precompiled headers
//
#include <mrpt/containers/yaml.h>
#include <mrpt/math/TLine2D.h>
#include <mrpt/math/TPolygon2D.h>
#include <mrpt/math/TPolygon3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TSegment2D.h>
#include <mrpt/math/epsilon.h>

#include <iostream>

#include "polygons_utils.h"

using namespace mrpt::math;

double TPolygon2D::distance(const TPoint2D& point) const
{
	if (contains(point)) return 0;
	std::vector<TSegment2D> sgs;
	getAsSegmentList(sgs);

	if (sgs.empty())
		THROW_EXCEPTION("Cannot compute distance to an empty polygon.");

	double distance = std::numeric_limits<double>::max();

	for (auto it = sgs.begin(); it != sgs.end(); ++it)
	{
		double d = (*it).distance(point);
		if (d < distance) distance = d;
	}
	return distance;
}

void TPolygon2D::getBoundingBox(
	TPoint2D& min_coords, TPoint2D& max_coords) const
{
	ASSERTMSG_(!this->empty(), "getBoundingBox() called on an empty polygon!");
	min_coords.x = min_coords.y = std::numeric_limits<double>::max();
	max_coords.x = max_coords.y = -std::numeric_limits<double>::max();
	for (size_t i = 0; i < size(); i++)
	{
		mrpt::keep_min(min_coords.x, (*this)[i].x);
		mrpt::keep_min(min_coords.y, (*this)[i].y);
		mrpt::keep_max(max_coords.x, (*this)[i].x);
		mrpt::keep_max(max_coords.y, (*this)[i].y);
	}
}

// isLeft(): tests if a point is Left|On|Right of an infinite line.
//    Input:  three points P0, P1, and P2
//    Return: >0 for P2 left of the line through P0 and P1
//            =0 for P2  on the line
//            <0 for P2  right of the line
//    See: Algorithm 1 "Area of Triangles and Polygons"
inline double isLeft(
	const mrpt::math::TPoint2D& P0, const mrpt::math::TPoint2D& P1,
	const mrpt::math::TPoint2D& P2)
{
	return ((P1.x - P0.x) * (P2.y - P0.y) - (P2.x - P0.x) * (P1.y - P0.y));
}

bool TPolygon2D::contains(const TPoint2D& P) const
{
	// References:
	// - http://geomalgorithms.com/a03-_inclusion.html
	// - https://en.wikipedia.org/wiki/Point_in_polygon#Winding_number_algorithm

	int wn = 0;	 // the  winding number counter

	// loop through all edges of the polygon
	const size_t n = this->size();
	for (size_t i = 0; i < n; i++)	// edge from V[i] to  V[i+1]
	{
		if ((*this)[i].y <= P.y)
		{
			// start y <= P.y
			if ((*this)[(i + 1) % n].y > P.y)  // an upward crossing
				if (isLeft((*this)[i], (*this)[(i + 1) % n], P) >
					0)	// P left of  edge
					++wn;  // have  a valid up intersect
		}
		else
		{
			// start y > P.y (no test needed)
			if ((*this)[(i + 1) % n].y <= P.y)	// a downward crossing
				if (isLeft((*this)[i], (*this)[(i + 1) % n], P) <
					0)	// P right of  edge
					--wn;  // have  a valid down intersect
		}
	}

	return wn != 0;
}
void TPolygon2D::getAsSegmentList(vector<TSegment2D>& v) const
{
	size_t N = size();
	v.resize(N);
	for (size_t i = 0; i < N - 1; i++)
		v[i] = TSegment2D(operator[](i), operator[](i + 1));
	v[N - 1] = TSegment2D(operator[](N - 1), operator[](0));
}

void TPolygon2D::generate3DObject(TPolygon3D& p) const
{
	p = TPolygon3D(*this);
}
void TPolygon2D::getCenter(TPoint2D& p) const
{
	for_each(begin(), end(), FAddPoint<TPoint2D, 2>(p));
	size_t N = size();
	p.x /= N;
	p.y /= N;
}
bool TPolygon2D::isConvex() const
{
	size_t N = size();
	if (N <= 3) return true;
	vector<TSegment2D> sgms;
	getAsSegmentList(sgms);
	for (size_t i = 0; i < N; i++)
	{
		char s = 0;
		auto l = TLine2D(sgms[i]);
		for (size_t j = 0; j < N; j++)
		{
			double d = l.evaluatePoint(operator[](j));
			if (std::abs(d) < getEpsilon()) continue;
			else if (!s)
				s = (d > 0) ? 1 : -1;
			else if (s != ((d > 0) ? 1 : -1))
				return false;
		}
	}
	return true;
}
void TPolygon2D::removeRepeatedVertices() { removeRepVertices(*this); }
void TPolygon2D::removeRedundantVertices()
{
	removeRepeatedVertices();
	removeUnusedVertices(*this);
}
void TPolygon2D::getPlotData(
	std::vector<double>& x, std::vector<double>& y) const
{
	size_t N = size();
	x.resize(N + 1);
	y.resize(N + 1);
	for (size_t i = 0; i < N; i++)
	{
		x[i] = operator[](i).x;
		y[i] = operator[](i).y;
	}
	x[N] = operator[](0).x;
	y[N] = operator[](0).y;
}
TPolygon2D::TPolygon2D(const TPolygon3D& p) : std::vector<TPoint2D>()
{
	size_t N = p.size();
	resize(N);
	for (size_t i = 0; i < N; i++)
		operator[](i) = TPoint2D(p[i]);
}
void TPolygon2D::createRegularPolygon(
	size_t numEdges, double radius, TPolygon2D& poly)
{
	if (numEdges < 3 || std::abs(radius) < getEpsilon())
		throw std::logic_error(
			"Invalid arguments for regular polygon creations");
	poly.resize(numEdges);
	for (size_t i = 0; i < numEdges; i++)
	{
		double angle = i * M_PI * 2 / numEdges;
		poly[i] = TPoint2D(radius * cos(angle), radius * sin(angle));
	}
}

inline void TPolygon2D::createRegularPolygon(
	size_t numEdges, double radius, TPolygon2D& poly, const TPose2D& pose)
{
	createRegularPolygon(numEdges, radius, poly);
	for (size_t i = 0; i < numEdges; i++)
		poly[i] = pose.composePoint(poly[i]);
}

std::ostream& mrpt::math::operator<<(std::ostream& o, const TPolygon2D& p)
{
	o << "mrpt::math::TPolygon2D vertices:\n";
	for (const auto& v : p)
		o << " - " << v << "\n";
	return o;
}

TPolygon2D TPolygon2D::FromYAML(const mrpt::containers::yaml& c)
{
	if (c.isNullNode() || c.empty()) return {};
	TPolygon2D p;
	ASSERT_(c.isSequence());
	for (const auto& vertex : c.asSequence())
	{
		ASSERT_(vertex.isSequence());
		const auto& vertexData = vertex.asSequence();
		ASSERT_EQUAL_(vertexData.size(), 2U);
		p.emplace_back(
			vertexData.at(0).as<double>(), vertexData.at(1).as<double>());
	}

	return p;
}

mrpt::containers::yaml TPolygon2D::asYAML() const
{
	mrpt::containers::yaml c = mrpt::containers::yaml::Sequence();

	for (const auto& vertex : *this)
	{
		auto pts = mrpt::containers::yaml::Sequence({vertex.x, vertex.y});
		pts.printInShortFormat = true;
		c.push_back(pts);
	}

	return c;
}
