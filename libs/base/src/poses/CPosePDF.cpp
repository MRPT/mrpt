/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/utils/CStream.h>

using namespace mrpt::poses;
using namespace mrpt::math;
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
	CMatrixDouble33			 &df_du,
	const bool compute_df_dx,
	const bool compute_df_du )

{
	const double   spx = sin(x.phi());
	const double   cpx = cos(x.phi());
	if (compute_df_dx)
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

		df_dx.get_unsafe(0,2) = -spx*xu-cpx*yu;
		df_dx.get_unsafe(1,2) =  cpx*xu-spx*yu;
	}

	if (compute_df_du)
	{
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
}


