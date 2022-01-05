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
#include <mrpt/math/TLine3D.h>
#include <mrpt/math/TObject2D.h>
#include <mrpt/math/TObject3D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPolygon2D.h>
#include <mrpt/math/TPolygon3D.h>
#include <mrpt/math/TSegment3D.h>
#include <mrpt/serialization/CArchive.h>  // impl of << operator
#include <mrpt/serialization/stl_serialization.h>  // >> of TPolygon2D

#include <sstream>

using namespace mrpt::math;

TObject3D TObject2D::generate3DObject() const
{
	if (isPoint()) { return {TPoint3D(getAs<TPoint2D>())}; }
	else if (isSegment())
	{
		return {TSegment3D(getAs<TSegment2D>())};
	}
	else if (isLine())
	{
		return {TLine3D(getAs<TLine2D>())};
	}
	else if (isPolygon())
	{
		return {TPolygon3D(getAs<TPolygon2D>())};
	}
	else if (empty())
	{
		return {};
	}
	THROW_EXCEPTION("Unexpected type.");
}

mrpt::serialization::CArchive& mrpt::math::operator>>(
	mrpt::serialization::CArchive& in, mrpt::math::TObject2D& o)
{
	// We cannot use ReadVariant<> since types are not CSerializable...sigh.

	switch (in.ReadAs<uint8_t>())
	{
		case 0: o.data.emplace<std::monostate>(); break;
		case 1:
			o.data.emplace<TPoint2D>();
			in >> o.getAs<TPoint2D>();
			break;
		case 2:
			o.data.emplace<TSegment2D>();
			in >> o.getAs<TSegment2D>();
			break;
		case 3:
			o.data.emplace<TLine2D>();
			in >> o.getAs<TLine2D>();
			break;
		case 4:
			o.data.emplace<TPolygon2D>();
			in >> o.getAs<TPolygon2D>();
			break;
		default: THROW_EXCEPTION("Unexpected type index");
	};
	return in;
}

mrpt::serialization::CArchive& mrpt::math::operator<<(
	mrpt::serialization::CArchive& out, const mrpt::math::TObject2D& o)
{
	if (o.empty()) { out.WriteAs<uint8_t>(0); }
	else if (o.isPoint())
	{
		out.WriteAs<uint8_t>(1);
		out << o.getAs<TPoint2D>();
	}
	else if (o.isSegment())
	{
		out.WriteAs<uint8_t>(2);
		out << o.getAs<TSegment2D>();
	}
	else if (o.isLine())
	{
		out.WriteAs<uint8_t>(3);
		out << o.getAs<TLine2D>();
	}
	else if (o.isPolygon())
	{
		out.WriteAs<uint8_t>(4);
		out << o.getAs<TPolygon2D>();
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
std::ostream& operator<<(std::ostream& o, const std::monostate&)
{
	o << "empty";
	return o;
}
}  // namespace mrpt::math

std::string TObject2D::asString() const
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
