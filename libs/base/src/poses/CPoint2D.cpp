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


#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>

using namespace mrpt::poses;
using namespace mrpt::utils;
using namespace mrpt::math;

IMPLEMENTS_SERIALIZABLE(CPoint2D, CPoint,mrpt::poses)


/*---------------------------------------------------------------
	Constructors
  ---------------------------------------------------------------*/
CPoint2D::CPoint2D(double x_,double y_)
{
	this->m_x		= x_;
	this->m_y		= y_;
	this->m_z		= 0;
	m_is3D = false;
}


/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
CPoint2D::CPoint2D( const CPose2D &o )
{
	this->m_x		= o.x();
	this->m_y		= o.y();
	this->m_z		= 0;
	m_is3D = false;
}

/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
CPoint2D::CPoint2D( const CPose3D &o )
{
	this->m_x		= o.x();
	this->m_y		= o.y();
	this->m_z		= 0;
	m_is3D = false;

}

/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void  CPoint2D::writeToStream(CStream &out, int *version) const
{
	if (version)
		*version = 1;
	else
	{
		// The coordinates:
		out << m_x << m_y;
	}
}

/*---------------------------------------------------------------
	Implements the reading from a CStream capability of
		CSerializable objects
  ---------------------------------------------------------------*/
void  CPoint2D::readFromStream(CStream &in, int version)
{
	switch(version)
	{
	case 0:
		{
			// The coordinates:
			float f;
			in >> f; m_x=f;
			in >> f; m_y=f;
		} break;
	case 1:
		{
			// The coordinates:
			in >> m_x >> m_y;
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
}

/**  Textual output stream function.
 */
std::ostream& mrpt::poses::operator << (std::ostream& o, const CPoint2D& p)
{
	o << "(" << p.x() << "," << p.y() <<")";
	return o;
}

/*---------------------------------------------------------------
The operator D="this"-b is the pose inverse compounding operator.
   The resulting pose "D" is the diference between this pose and "b"
 ---------------------------------------------------------------*/
CPoint2D  CPoint2D::operator - (const CPose2D& b) const
{
	double  ccos = cos(b.phi());
	double  ssin = sin(b.phi());


	return CPoint2D(
		 (m_x - b.x()) * ccos + (m_y - b.y()) * ssin,
		-(m_x - b.x()) * ssin + (m_y - b.y()) * ccos );
}

bool mrpt::poses::operator < (const CPoint2D &a, const CPoint2D &b)
{
	if (a.x()<b.x()) return true;
	else return a.y()<b.y();
}

CPoint2D::CPoint2D(const mrpt::math::TPoint2D &o)	{
	m_x=o.x;
	m_y=o.y;
	m_z=0;
	m_is3D=false;
}

CPoint2D::CPoint2D(const CPoint3D &o)	
{
	this->m_x		= o.x();
	this->m_y		= o.y();
	this->m_z		= 0;
	m_is3D = false;
}

 void CPoint2D::getAsVector(vector_double &v) const
 {
	 v.resize(2);
	 v[0]=m_x;
	 v[1]=m_y;
 }


bool mrpt::poses::operator==(const CPoint2D &p1,const CPoint2D &p2)
{
	return (p1.x()==p2.x())&&(p1.y()==p2.y());
}

bool mrpt::poses::operator!=(const CPoint2D &p1,const CPoint2D &p2)
{
	return (p1.x()!=p2.x())||(p1.y()!=p2.y());
}
