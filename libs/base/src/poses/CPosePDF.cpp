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

#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPosePDFGaussian.h>

using namespace mrpt::poses;
using namespace std;

IMPLEMENTS_VIRTUAL_SERIALIZABLE( CPosePDF, CSerializable, mrpt::poses )


void CPosePDF::jacobiansPoseComposition(
	const CPosePDFGaussian &x,
	const CPosePDFGaussian &u,
	CMatrixDouble33			 &df_dx,
	CMatrixDouble33			 &df_du)
{
	CPosePDF::jacobiansPoseComposition(x.mean,u.mean,df_dx,df_du);
}

/*---------------------------------------------------------------
					jacobiansPoseComposition
 ---------------------------------------------------------------*/
void CPosePDF::jacobiansPoseComposition(
	const CPose2D &x,
	const CPose2D &u,
	CMatrixDouble33			 &df_dx,
	CMatrixDouble33			 &df_du)
{
/*
	df_dx =
	[ 1, 0, -sin(phi_x)*x_u-cos(phi_x)*y_u ]
	[ 0, 1,  cos(phi_x)*x_u-sin(phi_x)*y_u ]
	[ 0, 0,                              1 ]
*/
	df_dx.unit(3,1.0);

	const double   xu = u.x();
	const double   yu = u.y();
	const double   spx = sin(x.phi());
	const double   cpx = cos(x.phi());

	df_dx.get_unsafe(0,2) = -spx*xu-cpx*yu;
	df_dx.get_unsafe(1,2) =  cpx*xu-spx*yu;

/*
	df_du =
	[ cos(phi_x) , -sin(phi_x) ,  0  ]
	[ sin(phi_x) ,  cos(phi_x) ,  0  ]
	[         0  ,          0  ,  1  ]
*/
	// This is the homogeneous matrix of "x":
	df_du.get_unsafe(0,2) =
	df_du.get_unsafe(1,2) =
	df_du.get_unsafe(2,0) =
	df_du.get_unsafe(2,1) = 0;
	df_du.get_unsafe(2,2) = 1;

	df_du.get_unsafe(0,0) =  cpx;
	df_du.get_unsafe(0,1) = -spx;
	df_du.get_unsafe(1,0) =  spx;
	df_du.get_unsafe(1,1) =  cpx;
}


