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

#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>

using namespace mrpt::poses;
using namespace mrpt::utils;
using namespace mrpt::math;

IMPLEMENTS_SERIALIZABLE(CPoint2D, CSerializable, mrpt::poses)

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
		out << m_coords[0] << m_coords[1];
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
			in >> f; m_coords[0]=f;
			in >> f; m_coords[1]=f;
		} break;
	case 1:
		{
			// The coordinates:
			in >> m_coords[0] >> m_coords[1];
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

/*---------------------------------------------------------------
The operator D="this"-b is the pose inverse compounding operator.
   The resulting pose "D" is the diference between this pose and "b"
 ---------------------------------------------------------------*/
CPoint2D  CPoint2D::operator - (const CPose2D& b) const
{
	const double  ccos = cos(b.phi());
	const double  ssin = sin(b.phi());
	const double  Ax = x()-b.x();
	const double  Ay = y()-b.y();

	return CPoint2D( Ax * ccos + Ay * ssin, -Ax * ssin + Ay * ccos );
}
