/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "poses-precomp.h"  // Precompiled headers

#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSchemeArchiveBase.h>
#include <limits>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::math;

IMPLEMENTS_SERIALIZABLE(CPoint3D, CSerializable, mrpt::poses)

/** Constructor from an CPoint2D object. */  // Here instead of in the .h to
// avoid headers include loops.
CPoint3D::CPoint3D(const CPoint2D& p)
{
	m_coords[0] = p.x();
	m_coords[1] = p.y();
	m_coords[2] = 0;
}
/** Constructor from an CPose2D object. */
CPoint3D::CPoint3D(const CPose2D& p)
{
	m_coords[0] = p.x();
	m_coords[1] = p.y();
	m_coords[2] = 0;
}

/** Constructor from an CPose3D object. */
CPoint3D::CPoint3D(const CPose3D& p)
{
	m_coords[0] = p.x();
	m_coords[1] = p.y();
	m_coords[2] = p.z();
}

uint8_t CPoint3D::serializeGetVersion() const { return 1; }
void CPoint3D::serializeTo(mrpt::serialization::CArchive& out) const
{
	out << m_coords[0] << m_coords[1] << m_coords[2];
}
void CPoint3D::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		{
			float f;
			in >> f;
			m_coords[0] = f;
			in >> f;
			m_coords[1] = f;
			in >> f;
			m_coords[2] = f;
		}
		break;
		case 1:
		{
			// The coordinates:
			in >> m_coords[0] >> m_coords[1] >> m_coords[2];
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}
/** Serialize CSerializable Object to CSchemeArchiveBase derived object*/ 
void CPoint3D::serializeTo(mrpt::serialization::CSchemeArchiveBase& out) const
{
	(*out["datatype"]) = std::string(this->GetRuntimeClass()->className);
	(*out["version"]) = 1;
	(*out["x"]) = m_coords[0];
	(*out["y"]) = m_coords[1];
	(*out["z"]) = m_coords[2];
}
/** Serialize CSchemeArchiveBase derived object to CSerializable Object*/
void CPoint3D::serializeFrom(mrpt::serialization::CSchemeArchiveBase& in)
{
	uint8_t version = static_cast<int>(*in["version"]);	//default is 0
	if(static_cast<std::string>(*in["datatype"]) == 
		std::string(this->GetRuntimeClass()->className)) //match the classname
	{
		switch(version)
		{
			case 1:
			{
				m_coords[0] = static_cast<double>(*in["x"]);
				m_coords[1] = static_cast<double>(*in["y"]);
				m_coords[2] = static_cast<double>(*in["z"]);
			}
			break;
			default:
				MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
		}
	}
}
/*---------------------------------------------------------------
				point3D = point3D - pose3D
  ---------------------------------------------------------------*/
CPoint3D CPoint3D::operator-(const CPose3D& b) const
{
	// JLBC: 7-FEB-2008: Why computing the whole matrix multiplication?? ;-)
	//   5.7us -> 4.1us -> 3.1us (with optimization of HM matrices by reference)
	// JLBC: 10-APR-2009: Usage of fixed-size 4x4 matrix, should be even faster
	// now.
	CMatrixDouble44 B_INV(UNINITIALIZED_MATRIX);
	b.getInverseHomogeneousMatrix(B_INV);

	return CPoint3D(
		B_INV.get_unsafe(0, 0) * m_coords[0] +
			B_INV.get_unsafe(0, 1) * m_coords[1] +
			B_INV.get_unsafe(0, 2) * m_coords[2] + B_INV.get_unsafe(0, 3),
		B_INV.get_unsafe(1, 0) * m_coords[0] +
			B_INV.get_unsafe(1, 1) * m_coords[1] +
			B_INV.get_unsafe(1, 2) * m_coords[2] + B_INV.get_unsafe(1, 3),
		B_INV.get_unsafe(2, 0) * m_coords[0] +
			B_INV.get_unsafe(2, 1) * m_coords[1] +
			B_INV.get_unsafe(2, 2) * m_coords[2] + B_INV.get_unsafe(2, 3));
}

/*---------------------------------------------------------------
				point3D = point3D - point3D
  ---------------------------------------------------------------*/
CPoint3D CPoint3D::operator-(const CPoint3D& b) const
{
	return CPoint3D(
		m_coords[0] - b.m_coords[0], m_coords[1] - b.m_coords[1],
		m_coords[2] - b.m_coords[2]);
}

/*---------------------------------------------------------------
				point3D = point3D + point3D
  ---------------------------------------------------------------*/
CPoint3D CPoint3D::operator+(const CPoint3D& b) const
{
	return CPoint3D(
		m_coords[0] + b.m_coords[0], m_coords[1] + b.m_coords[1],
		m_coords[2] + b.m_coords[2]);
}

/*---------------------------------------------------------------
				pose3D = point3D + pose3D
  ---------------------------------------------------------------*/
CPose3D CPoint3D::operator+(const CPose3D& b) const
{
	return CPose3D(
		m_coords[0] + b.x(), m_coords[1] + b.y(), m_coords[2] + b.z(), b.yaw(),
		b.pitch(), b.roll());
}

void CPoint3D::setToNaN()
{
	for (int i = 0; i < 3; i++)
		m_coords[i] = std::numeric_limits<double>::quiet_NaN();
}

mrpt::math::TPoint3D CPoint3D::asTPoint() const
{
	return mrpt::math::TPoint3D(x(), y(), z());
}
