/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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

IMPLEMENTS_SERIALIZABLE(CPoint3D, CPoint,mrpt::poses)

/*---------------------------------------------------------------
	Constructors
  ---------------------------------------------------------------*/
CPoint3D::CPoint3D(const double x_,const double y_,const double z_)
{
	this->m_x		= x_;
	this->m_y		= y_;
	this->m_z		= z_;
	m_is3D = true;
}

CPoint3D::CPoint3D( const CPoint2D &p)
{
	m_x  = p.x();
	m_y  = p.y();
	m_z  = p.z();
	m_is3D = true;
}

CPoint3D::CPoint3D( const CPose3D &p)
{
	m_x  = p.x();
	m_y  = p.y();
	m_z  = p.z();
	m_is3D = true;
}

CPoint3D::CPoint3D( const CPose2D &p)
{
	m_x  = p.x();
	m_y  = p.y();
	m_z  = 0;
	m_is3D = true;
}


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
		out << m_x << m_y << m_z;
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
			in >> f; m_x=f;
			in >> f; m_y=f;
			in >> f; m_z=f;
		} break;
	case 1:
		{
			// The coordinates:
			in >> m_x >> m_y >> m_z;
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

/**  Textual output stream function.
 */
std::ostream& poses::operator << (std::ostream& o, const CPoint3D& p)
{
	o << std::fixed << std::setprecision(3) << "(" << p.x() << "," << p.y() << "," << p.z() << ")";
	return o;
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
		B_INV.get_unsafe(0,0) * m_x + B_INV.get_unsafe(0,1) * m_y + B_INV.get_unsafe(0,2) * m_z + B_INV.get_unsafe(0,3),
		B_INV.get_unsafe(1,0) * m_x + B_INV.get_unsafe(1,1) * m_y + B_INV.get_unsafe(1,2) * m_z + B_INV.get_unsafe(1,3),
		B_INV.get_unsafe(2,0) * m_x + B_INV.get_unsafe(2,1) * m_y + B_INV.get_unsafe(2,2) * m_z + B_INV.get_unsafe(2,3) );
}

/*---------------------------------------------------------------
				point3D = point3D - point3D
  ---------------------------------------------------------------*/
CPoint3D  CPoint3D::operator - (const CPoint3D& b) const
{
	return CPoint3D( m_x-b.m_x, m_y-b.m_y, m_z-b.m_z );
}


/*---------------------------------------------------------------
				point3D = point3D + point3D
  ---------------------------------------------------------------*/
CPoint3D  CPoint3D::operator + (const CPoint3D& b) const
{
	return CPoint3D( m_x+b.m_x, m_y+b.m_y, m_z+b.m_z );
}

/*---------------------------------------------------------------
				pose3D = point3D + pose3D
  ---------------------------------------------------------------*/
CPose3D	CPoint3D::operator + (const CPose3D& b) const
{
	return CPose3D( m_x+b.x(), m_y+b.y(),m_z+b.z(), b.yaw(), b.pitch(), b.roll() );
}

CPoint3D::CPoint3D(const mrpt::math::TPoint3D &o)	{
	m_x=o.x;
	m_y=o.y;
	m_z=o.z;
	m_is3D=true;
}

 void CPoint3D::getAsVector(vector_double &v) const
 {
	 v.resize(3);
	 v[0]=m_x;
	 v[1]=m_y;
	 v[2]=m_z;
 }

bool mrpt::poses::operator==(const CPoint3D &p1,const CPoint3D &p2)
{
	return (p1.x()==p2.x())&&(p1.y()==p2.y())&&(p1.z()==p2.z());
}

bool mrpt::poses::operator!=(const CPoint3D &p1,const CPoint3D &p2)
{
	return (p1.x()!=p2.x())||(p1.y()!=p2.y())||(p1.z()!=p2.z());
}

