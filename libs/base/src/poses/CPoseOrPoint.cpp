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



#include <mrpt/poses/CPoseOrPoint.h>
#include <mrpt/math/utils.h>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::math;

IMPLEMENTS_VIRTUAL_SERIALIZABLE(CPoseOrPoint, CSerializable,mrpt::poses)

/*---------------------------------------------------------------
							norm
	Returns the euclidean norm of vector:
  ---------------------------------------------------------------*/
double  CPoseOrPoint::norm() const
{
	if (m_is3D)
			return sqrt(square(m_x)+square(m_y)+square(m_z));
	else	return sqrt(square(m_x)+square(m_y));
}

/*---------------------------------------------------------------
					getInverseHomogeneousMatrix
  ---------------------------------------------------------------*/
void CPoseOrPoint::getInverseHomogeneousMatrix(CMatrixDouble44 &out_HM)const
{
	CMatrixDouble44  ptrHM(UNINITIALIZED_MATRIX);
	getHomogeneousMatrix(ptrHM);
	math::homogeneousMatrixInverse( ptrHM, out_HM );
}
