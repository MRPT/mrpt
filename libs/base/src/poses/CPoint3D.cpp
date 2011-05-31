/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/base.h>  // Precompiled headers


#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPose2D.h>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::utils;

IMPLEMENTS_SERIALIZABLE(CPoint3D, CSerializable, mrpt::poses)

/** Constructor from an CPoint2D object. */  // Here instead of in the .h to avoid headers include loops.
CPoint3D::CPoint3D( const CPoint2D &p) { m_coords[0]=p.x(); m_coords[1]=p.y(); m_coords[2]=0; }
/** Constructor from an CPose2D object. */
CPoint3D::CPoint3D( const CPose2D &p) { m_coords[0]=p.x(); m_coords[1]=p.y(); m_coords[2]=0; }

/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void  CPoint3D::writeToStream(CStream &out,int *version) const
{
	if (version)
		*version = 1;
	else
	{
		// The coordinates:
		out << m_coords[0] << m_coords[1] << m_coords[2];
	}
}

/*---------------------------------------------------------------
	Implements the reading from a CStream capability of
		CSerializable objects
  ---------------------------------------------------------------*/
void  CPoint3D::readFromStream(CStream &in,int version)
{
	switch(version)
	{
	case 0:
		{
			float f;
			in >> f; m_coords[0]=f;
			in >> f; m_coords[1]=f;
			in >> f; m_coords[2]=f;
		} break;
	case 1:
		{
			// The coordinates:
			in >> m_coords[0] >> m_coords[1] >> m_coords[2];
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

/*---------------------------------------------------------------
				point3D = point3D - pose3D
  ---------------------------------------------------------------*/
CPoint3D  CPoint3D::operator - (const CPose3D& b) const
{
	// JLBC: 7-FEB-2008: Why computing the whole matrix multiplication?? ;-)
	//   5.7us -> 4.1us -> 3.1us (with optimization of HM matrices by reference)
	// JLBC: 10-APR-2009: Usage of fixed-size 4x4 matrix, should be even faster now.
	CMatrixDouble44 B_INV(UNINITIALIZED_MATRIX);
	b.getInverseHomogeneousMatrix( B_INV );

	return CPoint3D(
		B_INV.get_unsafe(0,0) * m_coords[0] + B_INV.get_unsafe(0,1) * m_coords[1] + B_INV.get_unsafe(0,2) * m_coords[2] + B_INV.get_unsafe(0,3),
		B_INV.get_unsafe(1,0) * m_coords[0] + B_INV.get_unsafe(1,1) * m_coords[1] + B_INV.get_unsafe(1,2) * m_coords[2] + B_INV.get_unsafe(1,3),
		B_INV.get_unsafe(2,0) * m_coords[0] + B_INV.get_unsafe(2,1) * m_coords[1] + B_INV.get_unsafe(2,2) * m_coords[2] + B_INV.get_unsafe(2,3) );
}

/*---------------------------------------------------------------
				point3D = point3D - point3D
  ---------------------------------------------------------------*/
CPoint3D  CPoint3D::operator - (const CPoint3D& b) const
{
	return CPoint3D( m_coords[0]-b.m_coords[0], m_coords[1]-b.m_coords[1], m_coords[2]-b.m_coords[2] );
}


/*---------------------------------------------------------------
				point3D = point3D + point3D
  ---------------------------------------------------------------*/
CPoint3D  CPoint3D::operator + (const CPoint3D& b) const
{
	return CPoint3D( m_coords[0]+b.m_coords[0], m_coords[1]+b.m_coords[1], m_coords[2]+b.m_coords[2] );
}

/*---------------------------------------------------------------
				pose3D = point3D + pose3D
  ---------------------------------------------------------------*/
CPose3D	CPoint3D::operator + (const CPose3D& b) const
{
	return CPose3D( m_coords[0]+b.x(), m_coords[1]+b.y(),m_coords[2]+b.z(), b.yaw(), b.pitch(), b.roll() );
}

