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


#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/math/utils.h>

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace mrpt::utils;

IMPLEMENTS_SERIALIZABLE(CPose2D, CSerializable ,mrpt::poses)

/*---------------------------------------------------------------
	Constructors
  ---------------------------------------------------------------*/
CPose2D::CPose2D() : m_phi(0)
{
	m_coords[0] =
	m_coords[1] = 0;
}

CPose2D::CPose2D(const double x,const double y,const double _phi) : m_phi(_phi)
{
	m_coords[0] = x;
	m_coords[1] = y;
	normalizePhi();
}

CPose2D::CPose2D(const CPoint2D &p) : m_phi(0)
{
	m_coords[0] = p.x();
	m_coords[1] = p.y();
}

CPose2D::CPose2D(const CPose3D &p) : m_phi(p.yaw())
{
	m_coords[0] = p.x();
	m_coords[1] = p.y();
}

/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void  CPose2D::writeToStream(CStream &out,int *version) const
{
	if (version)
		*version = 1;
	else
	{
		// The coordinates:
		out << m_coords[0] << m_coords[1] << m_phi;
	}
}

/*---------------------------------------------------------------
	Implements the reading from a CStream capability of
		CSerializable objects
  ---------------------------------------------------------------*/
void  CPose2D::readFromStream(CStream &in,int version)
{
	switch(version)
	{
	case 0:
		{
			// The coordinates:
			float x0,y0,phi0;
			in >> x0 >> y0 >> phi0;
			m_coords[0] = x0;
			m_coords[1] = y0;
			m_phi = phi0;
		} break;
	case 1:
		{
			// The coordinates:
			in >> m_coords[0] >> m_coords[1] >> m_phi;
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
}

/**  Textual output stream function.
 */
std::ostream& mrpt::poses::operator << (std::ostream& o, const CPose2D& p)
{
	o << format("(%.03f,%.03f,%.02fdeg)",p.x(),p.y(),RAD2DEG(p.phi()));
	return o;
}

/*---------------------------------------------------------------
The operator a="this"+D is the pose compounding operator.
 ---------------------------------------------------------------*/
CPose2D  CPose2D::operator + (const CPose2D& D)const
{
#ifdef HAVE_SINCOS
	double	ccos,ssin;
	::sincos(m_phi,&ssin,&ccos);
#else
	const double ccos = cos(m_phi);
	const double ssin = sin(m_phi);
#endif

	return CPose2D(
		m_coords[0] + D.m_coords[0] * ccos - D.m_coords[1] * ssin,
		m_coords[1] + D.m_coords[0] * ssin + D.m_coords[1] * ccos,
		m_phi + D.m_phi );
}

/*---------------------------------------------------------------
				composeFrom
 ---------------------------------------------------------------*/
void CPose2D::composeFrom(const CPose2D &A, const CPose2D &B)
{
#ifdef HAVE_SINCOS
	double	ccos,ssin;
	::sincos(A.m_phi,&ssin,&ccos);
#else
	const double ccos = cos(A.m_phi);
	const double ssin = sin(A.m_phi);
#endif
	// Use temporary variables for the cases (A==this) or (B==this)
	const double new_x = A.m_coords[0] + B.m_coords[0] * ccos - B.m_coords[1] * ssin;
	const double new_y = A.m_coords[1] + B.m_coords[0] * ssin + B.m_coords[1] * ccos;
	m_coords[0] = new_x;
	m_coords[1] = new_y;
	m_phi = math::wrapToPi(A.m_phi + B.m_phi);
}


/*---------------------------------------------------------------
The operator a="this"+D is the pose compounding operator.
 ---------------------------------------------------------------*/
CPose3D  CPose2D::operator + (const CPose3D& D) const
{
	return CPose3D(*this) + D;
}

/*---------------------------------------------------------------
The operator u'="this"+u is the pose/point compounding operator.
 ---------------------------------------------------------------*/
CPoint2D CPose2D::operator + (const CPoint2D& u)const
{
	const double ccos = cos(m_phi);
	const double ssin = sin(m_phi);

	return CPoint2D(
		m_coords[0] + u.x() * ccos - u.y() * ssin,
		m_coords[1] + u.x() * ssin + u.y() * ccos );
}

/** An alternative, slightly more efficient way of doing \f$ G = P \oplus L \f$ with G and L being 2D points and P this 2D pose.  */
void CPose2D::composePoint(double lx,double ly,double &gx, double &gy) const
{
	const double ccos = cos(m_phi);
	const double ssin = sin(m_phi);

	gx = m_coords[0] + lx * ccos - ly * ssin;
	gy = m_coords[1] + lx * ssin + ly * ccos;
}

/*---------------------------------------------------------------
The operator u'="this"+u is the pose/point compounding operator.
 ---------------------------------------------------------------*/
CPoint3D CPose2D::operator + (const CPoint3D& u)const
{
	double ccos = cos(m_phi);
	double ssin = sin(m_phi);

	return CPoint3D(
		m_coords[0] + u.x() * ccos - u.y() * ssin,
		m_coords[1] + u.x() * ssin + u.y() * ccos,
		u.z() );
}

/*---------------------------------------------------------------
The operator D="this"-b is the pose inverse compounding operator.
   The resulting pose "D" is the diference between this pose and "b"
 ---------------------------------------------------------------*/
void CPose2D::inverseComposeFrom(const CPose2D& A, const CPose2D& B )
{
	const double ccos = cos(B.m_phi);
	const double ssin = sin(B.m_phi);

	m_coords[0] = (A.m_coords[0] - B.m_coords[0]) * ccos + (A.m_coords[1] - B.m_coords[1]) * ssin;
	m_coords[1] =-(A.m_coords[0] - B.m_coords[0]) * ssin + (A.m_coords[1] - B.m_coords[1]) * ccos;
	m_phi = math::wrapToPi(A.m_phi - B.m_phi);
}

/*---------------------------------------------------------------
 Scalar sum of components: This is diferent from poses
   composition, which is implemented as "+" operators in "CPose" derived classes.
 ---------------------------------------------------------------*/
void CPose2D::AddComponents(CPose2D &p)
{
	m_coords[0]+=p.m_coords[0];
	m_coords[1]+=p.m_coords[1];
	m_phi+=p.m_phi;
}


/*---------------------------------------------------------------
 Scalar multiplication.
 ---------------------------------------------------------------*/
void CPose2D::operator *= (const double s)
{
	m_coords[0]*=s;
	m_coords[1]*=s;
	m_phi*=s;
}

/*---------------------------------------------------------------
	Returns the corresponding 4x4 homogeneous
	  transformation matrix for the point(translation),
	  or pose (translation+orientation).
---------------------------------------------------------------*/
void  CPose2D::getHomogeneousMatrix(CMatrixDouble44& m) const
{
	m.unit(4,1.0);

	m.set_unsafe(0,3, m_coords[0] );
	m.set_unsafe(1,3, m_coords[1] );

	const double ccos = cos(m_phi);
	const double csin = sin(m_phi);

	m.get_unsafe(0,0) = ccos;  m.get_unsafe(0,1)=-csin;
	m.get_unsafe(1,0) = csin;  m.get_unsafe(1,1)= ccos;
}

/** Forces "phi" to be in the range [-pi,pi];
*/
void  CPose2D::normalizePhi()
{
	m_phi = math::wrapToPi(m_phi);
}

CPose2D::CPose2D(const mrpt::math::TPose2D &o):m_phi(o.phi)	{
	m_coords[0]=o.x;
	m_coords[1]=o.y;
}

CPose2D::CPose2D(const CPoint3D &o): m_phi(0)	{
	this->m_coords[0]	= o.x();
	this->m_coords[1]	= o.y();
	normalizePhi();
}

/*---------------------------------------------------------------
		unary -
---------------------------------------------------------------*/
CPose2D mrpt::poses::operator -(const CPose2D&b)
{
	double ccos = cos(b.phi());
	double ssin = sin(b.phi());

	return CPose2D(
		-(b.x()) * ccos + (- b.y()) * ssin,
		-(-b.x()) * ssin + (- b.y()) * ccos,
		- b.phi() );
}

/*---------------------------------------------------------------
		getAsVector
---------------------------------------------------------------*/
void CPose2D::getAsVector(vector_double &v) const
{
	v.resize(3);
	v[0]=m_coords[0];
	v[1]=m_coords[1];
	v[2]=m_phi;
}

void CPose2D::getAsVector(mrpt::math::CArrayDouble<3> &v) const
{
	v[0]=m_coords[0];
	v[1]=m_coords[1];
	v[2]=m_phi;
}



bool mrpt::poses::operator==(const CPose2D &p1,const CPose2D &p2)
{
	return (p1.x()==p2.x())&&(p1.y()==p2.y())&&(p1.phi()==p2.phi());
}

bool mrpt::poses::operator!=(const CPose2D &p1,const CPose2D &p2)
{
	return (p1.x()!=p2.x())||(p1.y()!=p2.y())||(p1.phi()!=p2.phi());
}


TPoint2D mrpt::poses::operator +(const CPose2D &pose, const TPoint2D &u)
{
	double ccos = cos(pose.phi());
	double ssin = sin(pose.phi());
	return TPoint2D(
		pose.x() + u.x * ccos - u.y * ssin,
		pose.y() + u.x * ssin + u.y * ccos );
}

