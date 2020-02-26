/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "math-precomp.h"  // Precompiled headers

#include <mrpt/math/TLine3D.h>
#include <mrpt/math/TObject2D.h>
#include <mrpt/math/TObject3D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPolygon2D.h>
#include <mrpt/math/TPolygon3D.h>
#include <mrpt/math/TSegment3D.h>
#include <mrpt/serialization/CArchive.h>  // impl of << operator
#include <mrpt/serialization/stl_serialization.h>  // >> of TPolygon2D

using namespace mrpt::math;

void TObject2D::generate3DObject(TObject3D& obj) const
{
	switch (type)
	{
		case GEOMETRIC_TYPE_POINT:
			obj = TPoint3D(data.point);
			break;
		case GEOMETRIC_TYPE_SEGMENT:
			obj = TSegment3D(data.segment);
			break;
		case GEOMETRIC_TYPE_LINE:
			obj = TLine3D(data.line);
			break;
		case GEOMETRIC_TYPE_POLYGON:
			obj = TPolygon3D(*(data.polygon));
			break;
		default:
			obj = TObject3D();
			break;
	}
}
void TObject2D::getPoints(
	const std::vector<TObject2D>& objs, std::vector<TPoint2D>& pnts)
{
	for (const auto& obj : objs)
		if (obj.isPoint()) pnts.push_back(obj.data.point);
}
void TObject2D::getSegments(
	const std::vector<TObject2D>& objs, std::vector<TSegment2D>& sgms)
{
	for (const auto& obj : objs)
		if (obj.isSegment()) sgms.push_back(obj.data.segment);
}
void TObject2D::getLines(
	const std::vector<TObject2D>& objs, std::vector<TLine2D>& lins)
{
	for (const auto& obj : objs)
		if (obj.isLine()) lins.push_back(obj.data.line);
}
void TObject2D::getPolygons(
	const std::vector<TObject2D>& objs, std::vector<TPolygon2D>& polys)
{
	for (const auto& obj : objs)
		if (obj.isPolygon()) polys.push_back(*(obj.data.polygon));
}
void TObject2D::getPoints(
	const std::vector<TObject2D>& objs, std::vector<TPoint2D>& pnts,
	std::vector<TObject2D>& remainder)
{
	for (const auto& obj : objs)
		if (obj.isPoint())
			pnts.push_back(obj.data.point);
		else
			remainder.push_back(obj);
}
void TObject2D::getSegments(
	const std::vector<TObject2D>& objs, std::vector<TSegment2D>& sgms,
	std::vector<TObject2D>& remainder)
{
	for (const auto& obj : objs)
		if (obj.isSegment())
			sgms.push_back(obj.data.segment);
		else
			remainder.push_back(obj);
}
void TObject2D::getLines(
	const std::vector<TObject2D>& objs, std::vector<TLine2D>& lins,
	std::vector<TObject2D>& remainder)
{
	for (const auto& obj : objs)
		if (obj.isLine())
			lins.push_back(obj.data.line);
		else
			remainder.push_back(obj);
}
void TObject2D::getPolygons(
	const std::vector<TObject2D>& objs, std::vector<TPolygon2D>& polys,
	std::vector<TObject2D>& remainder)
{
	for (const auto& obj : objs)
		if (obj.isPolygon())
			polys.push_back(*(obj.data.polygon));
		else
			remainder.push_back(obj);
}

mrpt::serialization::CArchive& mrpt::math::operator>>(
	mrpt::serialization::CArchive& in, mrpt::math::TObject2D& o)
{
	uint16_t type;
	in >> type;
	switch (static_cast<unsigned char>(type))
	{
		case GEOMETRIC_TYPE_POINT:
		{
			TPoint2D p;
			in >> p;
			o = p;
		}
		break;
		case GEOMETRIC_TYPE_SEGMENT:
		{
			TSegment2D s;
			in >> s;
			o = s;
		}
		break;
		case GEOMETRIC_TYPE_LINE:
		{
			TLine2D l;
			in >> l;
			o = l;
		}
		break;
		case GEOMETRIC_TYPE_POLYGON:
		{
			TPolygon2D p;
			in >> p;
			o = p;
		}
		break;
		case GEOMETRIC_TYPE_UNDEFINED:
		{
			o = TObject2D();
		}
		break;
		default:
			throw std::logic_error(
				"Unknown TObject2D type found while reading stream");
	}
	return in;
}

mrpt::serialization::CArchive& mrpt::math::operator<<(
	mrpt::serialization::CArchive& out, const mrpt::math::TObject2D& o)
{
	out.WriteAs<uint16_t>(o.getType());
	switch (o.getType())
	{
		case GEOMETRIC_TYPE_POINT:
		{
			TPoint2D p;
			o.getPoint(p);
			return out << p;
		};
		case GEOMETRIC_TYPE_SEGMENT:
		{
			TSegment2D s;
			o.getSegment(s);
			return out << s;
		};
		case GEOMETRIC_TYPE_LINE:
		{
			TLine2D l;
			o.getLine(l);
			return out << l;
		};
		case GEOMETRIC_TYPE_POLYGON:
		{
			TPolygon2D p;
			o.getPolygon(p);
			return out << p;
		};
	}
	return out;
}
