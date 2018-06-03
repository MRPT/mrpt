/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "poses-precomp.h"  // Precompiled headers

#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSchemeArchiveBase.h>
#include <limits>

using namespace mrpt::poses;
using namespace mrpt::math;

IMPLEMENTS_SERIALIZABLE(CPoint2D, CSerializable, mrpt::poses)

uint8_t CPoint2D::serializeGetVersion() const { return 1; }
void CPoint2D::serializeTo(mrpt::serialization::CArchive& out) const
{
	out << m_coords[0] << m_coords[1];
}
void CPoint2D::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		{
			// The coordinates:
			float f;
			in >> f;
			m_coords[0] = f;
			in >> f;
			m_coords[1] = f;
		}
		break;
		case 1:
		{
			// The coordinates:
			in >> m_coords[0] >> m_coords[1];
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}
void CPoint2D::serializeTo(mrpt::serialization::CSchemeArchiveBase& out) const
{
	*out["x"] = m_coords[0];
	*out["y"] = m_coords[1];
}
void CPoint2D::serializeFrom(mrpt::serialization::CSchemeArchiveBase& in)
{
	m_coords[0] = (*in["x"]).asDouble();
	m_coords[1] = (*in["y"]).asDouble();
}
/*---------------------------------------------------------------
The operator D="this"-b is the pose inverse compounding operator.
   The resulting pose "D" is the diference between this pose and "b"
 ---------------------------------------------------------------*/
CPoint2D CPoint2D::operator-(const CPose2D& b) const
{
	const double ccos = cos(b.phi());
	const double ssin = sin(b.phi());
	const double Ax = x() - b.x();
	const double Ay = y() - b.y();

	return CPoint2D(Ax * ccos + Ay * ssin, -Ax * ssin + Ay * ccos);
}

void CPoint2D::setToNaN()
{
	for (int i = 0; i < 2; i++)
		m_coords[i] = std::numeric_limits<double>::quiet_NaN();
}

mrpt::math::TPoint2D CPoint2D::asTPoint() const
{
	return mrpt::math::TPoint2D(x(), y());
}
