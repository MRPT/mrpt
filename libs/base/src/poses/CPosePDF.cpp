/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
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


