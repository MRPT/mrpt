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
#include <mrpt/math/TObject2D.h>
#include <mrpt/math/TObject3D.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPolygon3D.h>
#include <mrpt/math/TSegment2D.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/stl_serialization.h>  // >> of TPolygon3D

#include <sstream>

using namespace mrpt::math;

TObject2D TObject3D::generate2DObject() const
{
	if (isPoint()) { return {TPoint2D(getAs<TPoint3D>())}; }
	else if (isSegment())
	{
		return {TSegment2D(getAs<TSegment3D>())};
	}
	else if (isLine())
	{
		return {TLine2D(getAs<TLine3D>())};
	}
	else if (isPolygon())
	{
		return {TPolygon2D(getAs<TPolygon3D>())};
	}
	else if (isPlane())
	{
		THROW_EXCEPTION("Cannot cast down a 3D plane to 2D.");
	}
	else if (empty())
	{
		return {};
	}
	THROW_EXCEPTION("Unexpected type.");
}

mrpt::serialization::CArchive& mrpt::math::operator>>(
	mrpt::serialization::CArchive& in, mrpt::math::TObject3D& o)
{
	// We cannot use ReadVariant<> since types are not CSerializable...sigh.

	switch (in.ReadAs<uint8_t>())
	{
		case 0: o.data.emplace<std::monostate>(); break;
		case 1:
			o.data.emplace<TPoint3D>();
			in >> o.getAs<TPoint3D>();
			break;
		case 2:
			o.data.emplace<TSegment3D>();
			in >> o.getAs<TSegment3D>();
			break;
		case 3:
			o.data.emplace<TLine3D>();
			in >> o.getAs<TLine3D>();
			break;
		case 4:
			o.data.emplace<TPolygon3D>();
			in >> o.getAs<TPolygon3D>();
			break;
		case 5:
			o.data.emplace<TPlane>();
			in >> o.getAs<TPlane>();
			break;
		default: THROW_EXCEPTION("Unexpected type index");
	};
	return in;
}

mrpt::serialization::CArchive& mrpt::math::operator<<(
	mrpt::serialization::CArchive& out, const mrpt::math::TObject3D& o)
{
	if (o.empty()) { out.WriteAs<uint8_t>(0); }
	else if (o.isPoint())
	{
		out.WriteAs<uint8_t>(1);
		out << o.getAs<TPoint3D>();
	}
	else if (o.isSegment())
	{
		out.WriteAs<uint8_t>(2);
		out << o.getAs<TSegment3D>();
	}
	else if (o.isLine())
	{
		out.WriteAs<uint8_t>(3);
		out << o.getAs<TLine3D>();
	}
	else if (o.isPolygon())
	{
		out.WriteAs<uint8_t>(4);
		out << o.getAs<TPolygon3D>();
	}
	else if (o.isPlane())
	{
		out.WriteAs<uint8_t>(5);
		out << o.getAs<TPlane>();
	}
	THROW_EXCEPTION("Unexpected type index");
	return out;
}

namespace mrpt::typemeta
{
// Specialization must occur in the same namespace
MRPT_DECLARE_TTYPENAME_NO_NAMESPACE(monostate, std)
}  // namespace mrpt::typemeta

namespace mrpt::math
{
extern std::ostream& operator<<(std::ostream& o, const std::monostate&);
}  // namespace mrpt::math

std::string TObject3D::asString() const
{
	std::stringstream ss;
	std::visit(
		[&](const auto& o) {
			ss << mrpt::typemeta::TTypeName<std::decay_t<decltype(o)>>::get()
			   << ": " << o;
		},
		data);
	return ss.str();
}
