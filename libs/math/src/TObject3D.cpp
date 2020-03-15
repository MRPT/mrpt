/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "math-precomp.h"  // Precompiled headers

#include <mrpt/math/TObject2D.h>
#include <mrpt/math/TObject3D.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPolygon3D.h>
#include <mrpt/math/TSegment2D.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/stl_serialization.h>  // >> of TPolygon3D

using namespace mrpt::math;

void TObject3D::generate2DObject(TObject2D& obj) const
{
	switch (type)
	{
		case GEOMETRIC_TYPE_POINT:
			obj = TPoint2D(data.point);
			break;
		case GEOMETRIC_TYPE_SEGMENT:
			obj = TSegment2D(data.segment);
			break;
		case GEOMETRIC_TYPE_LINE:
			obj = TLine2D(data.line);
			break;
		case GEOMETRIC_TYPE_POLYGON:
			obj = TPolygon2D(*(data.polygon));
			break;
		case GEOMETRIC_TYPE_PLANE:
			throw std::logic_error("Too many dimensions");
		default:
			obj = TObject2D();
			break;
	}
}

void TObject3D::getPoints(
	const std::vector<TObject3D>& objs, std::vector<TPoint3D>& pnts)
{
	for (const auto& obj : objs)
		if (obj.isPoint()) pnts.push_back(obj.data.point);
}
void TObject3D::getSegments(
	const std::vector<TObject3D>& objs, std::vector<TSegment3D>& sgms)
{
	for (const auto& obj : objs)
		if (obj.isSegment()) sgms.push_back(obj.data.segment);
}
void TObject3D::getLines(
	const std::vector<TObject3D>& objs, std::vector<TLine3D>& lins)
{
	for (const auto& obj : objs)
		if (obj.isLine()) lins.push_back(obj.data.line);
}
void TObject3D::getPlanes(
	const std::vector<TObject3D>& objs, std::vector<TPlane>& plns)
{
	for (const auto& obj : objs)
		if (obj.isPlane()) plns.push_back(obj.data.plane);
}
void TObject3D::getPolygons(
	const std::vector<TObject3D>& objs, std::vector<TPolygon3D>& polys)
{
	for (const auto& obj : objs)
		if (obj.isPolygon()) polys.push_back(*(obj.data.polygon));
}
void TObject3D::getPoints(
	const std::vector<TObject3D>& objs, std::vector<TPoint3D>& pnts,
	std::vector<TObject3D>& remainder)
{
	for (const auto& obj : objs)
		if (obj.isPoint())
			pnts.push_back(obj.data.point);
		else
			remainder.push_back(obj);
}
void TObject3D::getSegments(
	const std::vector<TObject3D>& objs, std::vector<TSegment3D>& sgms,
	std::vector<TObject3D>& remainder)
{
	for (const auto& obj : objs)
		if (obj.isSegment())
			sgms.push_back(obj.data.segment);
		else
			remainder.push_back(obj);
}
void TObject3D::getLines(
	const std::vector<TObject3D>& objs, std::vector<TLine3D>& lins,
	std::vector<TObject3D>& remainder)
{
	for (const auto& obj : objs)
		if (obj.isLine())
			lins.push_back(obj.data.line);
		else
			remainder.push_back(obj);
}
void TObject3D::getPlanes(
	const std::vector<TObject3D>& objs, std::vector<TPlane>& plns,
	std::vector<TObject3D>& remainder)
{
	for (const auto& obj : objs)
		if (obj.isPlane())
			plns.push_back(obj.data.plane);
		else
			remainder.push_back(obj);
}
void TObject3D::getPolygons(
	const std::vector<TObject3D>& objs, std::vector<TPolygon3D>& polys,
	std::vector<TObject3D>& remainder)
{
	for (const auto& obj : objs)
		if (obj.isPolygon())
			polys.push_back(*(obj.data.polygon));
		else
			remainder.push_back(obj);
}

mrpt::serialization::CArchive& mrpt::math::operator>>(
	mrpt::serialization::CArchive& in, mrpt::math::TObject3D& o)
{
	uint16_t type;
	in >> type;
	switch (static_cast<unsigned char>(type))
	{
		case GEOMETRIC_TYPE_POINT:
		{
			TPoint3D p;
			in >> p;
			o = p;
		}
		break;
		case GEOMETRIC_TYPE_SEGMENT:
		{
			TSegment3D s;
			in >> s;
			o = s;
		}
		break;
		case GEOMETRIC_TYPE_LINE:
		{
			TLine3D l;
			in >> l;
			o = l;
		}
		break;
		case GEOMETRIC_TYPE_PLANE:
		{
			TPlane p;
			in >> p;
			o = p;
		}
		break;
		case GEOMETRIC_TYPE_POLYGON:
		{
			TPolygon3D p;
			in >> p;
			o = p;
		}
		break;
		case GEOMETRIC_TYPE_UNDEFINED:
		{
			o = TObject3D();
		}
		break;
		default:
			throw std::logic_error(
				"Unknown TObject3D type found while reading stream");
	}
	return in;
}

mrpt::serialization::CArchive& mrpt::math::operator<<(
	mrpt::serialization::CArchive& out, const mrpt::math::TObject3D& o)
{
	out.WriteAs<uint16_t>(o.getType());
	switch (o.getType())
	{
		case GEOMETRIC_TYPE_POINT:
		{
			TPoint3D p;
			o.getPoint(p);
			return out << p;
		};
		case GEOMETRIC_TYPE_SEGMENT:
		{
			TSegment3D s;
			o.getSegment(s);
			return out << s;
		};
		case GEOMETRIC_TYPE_LINE:
		{
			TLine3D l;
			o.getLine(l);
			return out << l;
		};
		case GEOMETRIC_TYPE_PLANE:
		{
			TPlane p;
			o.getPlane(p);
			return out << p;
		};
		case GEOMETRIC_TYPE_POLYGON:
		{
			TPolygon3D p;
			o.getPolygon(p);
			return out << p;
		};
	}
	return out;
}
