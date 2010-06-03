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


#include <mrpt/poses/CPose.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPoint3D.h>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_VIRTUAL_SERIALIZABLE(CPose, CPoseOrPoint,mrpt::poses)


/*---------------------------------------------------------------
				pose3D = pose*D - pose3D
  ---------------------------------------------------------------*/
CPose3D  CPose::operator - (const CPose3D& b) const
{
	CMatrixDouble44 B_INV(UNINITIALIZED_MATRIX);
	b.getInverseHomogeneousMatrix( B_INV );

	CMatrixDouble44 HM(UNINITIALIZED_MATRIX);
	getHomogeneousMatrix(HM);

	CMatrixDouble44 RES(UNINITIALIZED_MATRIX);
	RES.multiply(B_INV,HM);

	return CPose3D( RES );
}
