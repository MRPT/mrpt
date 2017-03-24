/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "tfest-precomp.h"  // Precompiled headers

#include <mrpt/tfest/se2.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/random.h>

#if MRPT_HAS_SSE2
	#include <mrpt/utils/SSE_types.h>
#endif

using namespace mrpt;
using namespace mrpt::tfest;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;

// Wrapper:
bool tfest::se2_l2(
	const TMatchingPairList & in_correspondences,
	CPosePDFGaussian        & out_transformation )
{
	mrpt::math::TPose2D p;
	const bool ret = tfest::se2_l2(in_correspondences,p, &out_transformation.cov );
	out_transformation.mean = p;
	return ret;
}

/*---------------------------------------------------------------
			leastSquareErrorRigidTransformation

   Compute the best transformation (x,y,phi) given a set of
    correspondences between 2D points in two different maps.
   This method is intensively used within ICP.
  ---------------------------------------------------------------*/
bool  tfest::se2_l2(
	const TMatchingPairList & in_correspondences,
	TPose2D                 & out_transformation,
	CMatrixDouble33         * out_estimateCovariance )
{
	MRPT_START

	const size_t N = in_correspondences.size();

	if (N<2) return false;

	const float N_inv = 1.0f/N;  // For efficiency, keep this value.

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
#if MRPT_HAS_SSE2
	// SSE vectorized version:

	//{
	//	TMatchingPair dumm;
	//	MRPT_COMPILE_TIME_ASSERT(sizeof(dumm.this_x)==sizeof(float))
	//	MRPT_COMPILE_TIME_ASSERT(sizeof(dumm.other_x)==sizeof(float))
	//}

	__m128  sum_a_xyz = _mm_setzero_ps(); // All 4 zeros (0.0f)
	__m128  sum_b_xyz = _mm_setzero_ps(); // All 4 zeros (0.0f)

	//   [ f0     f1      f2      f3  ]
	//    xa*xb  ya*yb   xa*yb  xb*ya
	__m128  sum_ab_xyz = _mm_setzero_ps(); // All 4 zeros (0.0f)

	for (TMatchingPairList::const_iterator corrIt=in_correspondences.begin(); corrIt!=in_correspondences.end(); corrIt++)
	{
		// Get the pair of points in the correspondence:
		//   a_xyyx = [   xa     ay   |   xa    ya ]
		//   b_xyyx = [   xb     yb   |   yb    xb ]
		//      (product)
		//            [  xa*xb  ya*yb   xa*yb  xb*ya
		//                LO0    LO1     HI2    HI3
		// Note: _MM_SHUFFLE(hi3,hi2,lo1,lo0)
		const __m128 a_xyz = _mm_loadu_ps( &corrIt->this_x );  // *Unaligned* load
		const __m128 b_xyz = _mm_loadu_ps( &corrIt->other_x );  // *Unaligned* load

		const __m128 a_xyxy = _mm_shuffle_ps(a_xyz,a_xyz, _MM_SHUFFLE(1,0,1,0) );
		const __m128 b_xyyx = _mm_shuffle_ps(b_xyz,b_xyz, _MM_SHUFFLE(0,1,1,0) );

		// Compute the terms:
		sum_a_xyz = _mm_add_ps(sum_a_xyz,a_xyz);
		sum_b_xyz = _mm_add_ps(sum_b_xyz,b_xyz);

		//   [ f0     f1      f2      f3  ]
		//    xa*xb  ya*yb   xa*yb  xb*ya
		sum_ab_xyz = _mm_add_ps(sum_ab_xyz, _mm_mul_ps(a_xyxy,b_xyyx));

	}

	MRPT_ALIGN16 float sums_a[4], sums_b[4];
	_mm_store_ps(sums_a,sum_a_xyz);
	_mm_store_ps(sums_b,sum_b_xyz);

	const float &SumXa=sums_a[0];
	const float &SumYa=sums_a[1];
	const float &SumXb=sums_b[0];
	const float &SumYb=sums_b[1];

	// Compute all four means:
	const __m128 Ninv_4val = _mm_set1_ps(N_inv); // load 4 copies of the same value
	sum_a_xyz = _mm_mul_ps(sum_a_xyz,Ninv_4val);
	sum_b_xyz = _mm_mul_ps(sum_b_xyz,Ninv_4val);


	// means_a[0]: mean_x_a
	// means_a[1]: mean_y_a
	// means_b[0]: mean_x_b
	// means_b[1]: mean_y_b
	MRPT_ALIGN16 float means_a[4], means_b[4];
	_mm_store_ps(means_a,sum_a_xyz);
	_mm_store_ps(means_b,sum_b_xyz);

	const float	&mean_x_a = means_a[0];
	const float	&mean_y_a = means_a[1];
	const float	&mean_x_b = means_b[0];
	const float	&mean_y_b = means_b[1];

	//      Sxx   Syy     Sxy    Syx
	//    xa*xb  ya*yb   xa*yb  xb*ya
	MRPT_ALIGN16 float cross_sums[4];
	_mm_store_ps(cross_sums,sum_ab_xyz);

	const float	&Sxx = cross_sums[0];
	const float	&Syy = cross_sums[1];
	const float	&Sxy = cross_sums[2];
	const float	&Syx = cross_sums[3];

	// Auxiliary variables Ax,Ay:
	const float Ax = N*(Sxx + Syy) - SumXa*SumXb - SumYa*SumYb;
	const float Ay = SumXa * SumYb + N*(Syx-Sxy)- SumXb * SumYa;

#else
	// Non vectorized version:
	float SumXa=0, SumXb=0, SumYa=0, SumYb=0;
	float Sxx=0, Sxy=0, Syx=0, Syy=0;

	for (TMatchingPairList::const_iterator corrIt=in_correspondences.begin(); corrIt!=in_correspondences.end(); corrIt++)
	{
		// Get the pair of points in the correspondence:
		const float xa = corrIt->this_x;
		const float ya = corrIt->this_y;
		const float xb = corrIt->other_x;
		const float yb = corrIt->other_y;

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

	const float	mean_x_a = SumXa * N_inv;
	const float	mean_y_a = SumYa * N_inv;
	const float	mean_x_b = SumXb * N_inv;
	const float	mean_y_b = SumYb * N_inv;

	// Auxiliary variables Ax,Ay:
	const float Ax = N*(Sxx + Syy) - SumXa*SumXb - SumYa*SumYb;
	const float Ay = SumXa * SumYb + N*(Syx-Sxy)- SumXb * SumYa;

#endif


	out_transformation.phi = (Ax!=0 || Ay!=0) ? atan2( static_cast<double>(Ay), static_cast<double>(Ax)) : 0.0;

	const double ccos = cos( out_transformation.phi );
	const double csin = sin( out_transformation.phi );

	out_transformation.x = mean_x_a - mean_x_b * ccos + mean_y_b * csin;
	out_transformation.y = mean_y_a - mean_x_b * csin - mean_y_b * ccos;

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

		C->get_unsafe(0,0) = 2.0*N_inv + BETA * square((mean_x_b*Ay+mean_y_b*Ax)/D);
		C->get_unsafe(1,1) = 2.0*N_inv + BETA * square((mean_x_b*Ax-mean_y_b*Ay)/D);
		C->get_unsafe(2,2) = BETA / D;

		C->get_unsafe(0,1) =
		C->get_unsafe(1,0) = -BETA*(mean_x_b*Ay+mean_y_b*Ax)*(mean_x_b*Ax-mean_y_b*Ay)/square(D);

		C->get_unsafe(0,2) =
		C->get_unsafe(2,0) = BETA*(mean_x_b*Ay+mean_y_b*Ax)/pow(D,1.5);

		C->get_unsafe(1,2) =
		C->get_unsafe(2,1) = BETA*(mean_y_b*Ay-mean_x_b*Ax)/pow(D,1.5);
	}

	return true;

	MRPT_END
}

