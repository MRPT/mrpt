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

#include <mrpt/scanmatching.h>  // Precompiled headers


#include <mrpt/scanmatching/scan_matching.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPosePDFSOG.h>
#include <mrpt/random.h>
#include <mrpt/math/CMatrixD.h>
#include <mrpt/math/utils.h>
#include <mrpt/math/CQuaternion.h>

#include <algorithm>

using namespace mrpt;
using namespace mrpt::scanmatching;
using namespace mrpt::random;
using namespace mrpt::utils;
using namespace std;


bool scanmatching::leastSquareErrorRigidTransformation(
	TMatchingPairList	&in_correspondences,
	CPosePDFGaussian				&out_transformation )
{
	return leastSquareErrorRigidTransformation(in_correspondences,out_transformation.mean, &out_transformation.cov );
}

/*---------------------------------------------------------------
			leastSquareErrorRigidTransformation

   Compute the best transformation (x,y,phi) given a set of
    correspondences between 2D points in two different maps.
   This method is intensively used within ICP.
  ---------------------------------------------------------------*/
bool  scanmatching::leastSquareErrorRigidTransformation(
	TMatchingPairList	&in_correspondences,
	CPose2D								&out_transformation,
	CMatrixDouble33						*out_estimateCovariance )
{
	MRPT_START;

	const size_t N = in_correspondences.size();

	if (N<2) return false;

	const double N_inv = 1.0/N;  // For efficiency, keep this value.

	// ----------------------------------------------------------------------
	// Compute the estimated pose. Notation from the paper:
	// "Mobile robot motion estimation by 2d scan matching with genetic and iterative
	// closest point algorithms", J.L. Martinez Rodriguez, A.J. Gonzalez, J. Morales
	// Rodriguez, A. Mandow Andaluz, A. J. Garcia Cerezo,
	// Journal of Field Robotics, 2006.
	// ----------------------------------------------------------------------

	// ----------------------------------------------------------------------
	//  For the formulas of the covariance, see:
	//   http://www.mrpt.org/Paper:Occupancy_Grid_Matching
	//   and Jose Luis Blanco's PhD thesis.
	// ----------------------------------------------------------------------
	double SumXa=0, SumXb=0, SumYa=0, SumYb=0;
	double Sxx=0, Sxy=0, Syx=0, Syy=0;

	for (TMatchingPairList::const_iterator corrIt=in_correspondences.begin(); corrIt!=in_correspondences.end(); corrIt++)
	{
		// Get the pair of points in the correspondence:
		const double xa = corrIt->this_x;
		const double ya = corrIt->this_y;
		const double xb = corrIt->other_x;
		const double yb = corrIt->other_y;

		// Compute the terms:
		SumXa+=xa;
		SumYa+=ya;

		SumXb += xb;
		SumYb += yb;

		Sxx += xa * xb;
		Sxy += xa * yb;
		Syx += ya * xb;
		Syy += ya * yb;
	}	// End of "for all correspondences"...

	const double	mean_x_a = SumXa * N_inv;
	const double	mean_y_a = SumYa * N_inv;
	const double	mean_x_b = SumXb * N_inv;
	const double	mean_y_b = SumYb * N_inv;

	// Auxiliary variables Ax,Ay:
	const double Ax = N*(Sxx + Syy) - SumXa*SumXb - SumYa*SumYb;
	const double Ay = SumXa * SumYb + N*(Syx-Sxy)- SumXb * SumYa;

	if (Ax!=0 || Ay!=0)
			out_transformation.phi( atan2( Ay, Ax) );
	else	out_transformation.phi(0);

	const double ccos = cos( out_transformation.phi() );
	const double csin = sin( out_transformation.phi() );

	out_transformation.x( mean_x_a - mean_x_b * ccos + mean_y_b * csin  );
	out_transformation.y( mean_y_a - mean_x_b * csin - mean_y_b * ccos );

	if ( out_estimateCovariance )
	{
		CMatrixDouble33  *C = out_estimateCovariance;  // less typing!

		// Compute the normalized covariance matrix:
		// -------------------------------------------
		double var_x_a = 0,var_y_a = 0,var_x_b = 0,var_y_b = 0;
		const double N_1_inv = 1.0/(N-1);

		// 0) Precompute the unbiased variances estimations:
		// ----------------------------------------------------
		for (TMatchingPairList::const_iterator corrIt=in_correspondences.begin(); corrIt!=in_correspondences.end(); corrIt++)
		{
			var_x_a += square( corrIt->this_x - mean_x_a );
			var_y_a += square( corrIt->this_y - mean_y_a );
			var_x_b += square( corrIt->other_x - mean_x_b );
			var_y_b += square( corrIt->other_y - mean_y_b );
		}
		var_x_a *= N_1_inv; //  /= (N-1)
		var_y_a *= N_1_inv;
		var_x_b *= N_1_inv;
		var_y_b *= N_1_inv;

		// 1) Compute  BETA = s_Delta^2 / s_p^2
		// --------------------------------
		const double BETA = (var_x_a+var_y_a+var_x_b+var_y_b)*pow(static_cast<double>(N),2.0)*static_cast<double>(N-1);

		// 2) And the final covariance matrix:
		//  (remember: this matrix has yet to be
		//   multiplied by var_p to be the actual covariance!)
		// -------------------------------------------------------
		const double D = square(Ax)+square(Ay);

		C->get_unsafe(0,0) = 2.0*N_inv + BETA * pow((mean_x_b*Ay+mean_y_b*Ax)/D,2.0);
		C->get_unsafe(1,1) = 2.0*N_inv + BETA * pow((mean_x_b*Ax-mean_y_b*Ay)/D,2.0);
		C->get_unsafe(2,2) = BETA / D;

		C->get_unsafe(0,1) =
		C->get_unsafe(1,0) = -BETA*(mean_x_b*Ay+mean_y_b*Ax)*(mean_x_b*Ax-mean_y_b*Ay)/square(D);

		C->get_unsafe(0,2) =
		C->get_unsafe(2,0) = BETA*(mean_x_b*Ay+mean_y_b*Ax)/pow(D,1.5);

		C->get_unsafe(1,2) =
		C->get_unsafe(2,1) = BETA*(mean_y_b*Ay-mean_x_b*Ax)/pow(D,1.5);

//		// 2) Compute  the jacobian J_Delta
//		// -----------------------------------
//		const double K_1 = square(Ax)+square(Ay);
//		const double K_32 = pow(K_1,1.5);
//		CMatrixFixedNumeric<double,3,2>		J;
//
//		J(0,0) = -( mean_x_b * Ay*Ay + mean_y_b * Ax * Ay )/K_32;
//		J(0,1) =  ( mean_x_b * Ax*Ay + mean_y_b * Ax * Ax )/K_32;
//		J(1,0) =  ( mean_x_b * Ax*Ay - mean_y_b * Ay * Ay )/K_32;
//		J(1,1) = -( mean_x_b * Ax*Ax - mean_y_b * Ax * Ay )/K_32;
//		J(2,0) = - Ay / K_1;
//		J(2,1) =   Ax / K_1;
//
//		// 3) And the final covariance matrix:
//		// -------------------------------------
//		out_estimateCovariance->multiply_AAt(J);	// = J * (~J);
//		(*out_estimateCovariance) *= var_Delta;
//		out_estimateCovariance->get_unsafe(0,0) += 2.0/square(N);
//		out_estimateCovariance->get_unsafe(1,1) += 2.0/square(N);
	}

	return true;

	MRPT_END;
}

