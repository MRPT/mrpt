/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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

