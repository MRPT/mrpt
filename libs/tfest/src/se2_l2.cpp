/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "tfest-precomp.h"	// Precompiled headers
//
#include <mrpt/core/cpu.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/random.h>
#include <mrpt/tfest/se2.h>

#include "se2_l2_internal.h"

using namespace mrpt;
using namespace mrpt::tfest;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;

// Wrapper:
bool tfest::se2_l2(
	const TMatchingPairList& in_correspondences,
	CPosePDFGaussian& out_transformation)
{
	mrpt::math::TPose2D p;
	const bool ret =
		tfest::se2_l2(in_correspondences, p, &out_transformation.cov);
	out_transformation.mean = CPose2D(p);
	return ret;
}

// Non-vectorized version
static mrpt::tfest::internal::se2_l2_impl_return_t<float> se2_l2_impl(
	const TMatchingPairList& in_correspondences)
{
	// SSE vectorized version:
	const size_t N = in_correspondences.size();
	ASSERT_(N >= 2);
	const float N_inv = 1.0f / N;  // For efficiency, keep this value.

	// Non vectorized version:
	float SumXa = 0, SumXb = 0, SumYa = 0, SumYb = 0;
	float Sxx = 0, Sxy = 0, Syx = 0, Syy = 0;

	for (const auto& p : in_correspondences)
	{
		// Get the pair of points in the correspondence:
		const float xa = p.this_x;
		const float ya = p.this_y;
		const float xb = p.other_x;
		const float yb = p.other_y;

		// Compute the terms:
		SumXa += xa;
		SumYa += ya;

		SumXb += xb;
		SumYb += yb;

		Sxx += xa * xb;
		Sxy += xa * yb;
		Syx += ya * xb;
		Syy += ya * yb;
	}  // End of "for all correspondences"...

	mrpt::tfest::internal::se2_l2_impl_return_t<float> ret;
	ret.mean_x_a = SumXa * N_inv;
	ret.mean_y_a = SumYa * N_inv;
	ret.mean_x_b = SumXb * N_inv;
	ret.mean_y_b = SumYb * N_inv;

	// Auxiliary variables Ax,Ay:
	ret.Ax = N * (Sxx + Syy) - SumXa * SumXb - SumYa * SumYb;
	ret.Ay = SumXa * SumYb + N * (Syx - Sxy) - SumXb * SumYa;

	return ret;
}

/*---------------------------------------------------------------
			leastSquareErrorRigidTransformation

   Compute the best transformation (x,y,phi) given a set of
	correspondences between 2D points in two different maps.
   This method is intensively used within ICP.
  ---------------------------------------------------------------*/
bool tfest::se2_l2(
	const TMatchingPairList& in_correspondences, TPose2D& out_transformation,
	CMatrixDouble33* out_estimateCovariance)
{
	MRPT_START

	const size_t N = in_correspondences.size();

	if (N < 2) return false;

	const float N_inv = 1.0f / N;  // For efficiency, keep this value.

	// ----------------------------------------------------------------------
	// Compute the estimated pose. Notation from the paper:
	// "Mobile robot motion estimation by 2d scan matching with genetic and
	// iterative
	// closest point algorithms", J.L. Martinez Rodriguez, A.J. Gonzalez, J.
	// Morales Rodriguez, A. Mandow Andaluz, A. J. Garcia Cerezo, Journal of
	// Field Robotics, 2006.
	// ----------------------------------------------------------------------

	// ----------------------------------------------------------------------
	//  For the formulas of the covariance, see:
	//   https://www.mrpt.org/Paper:Occupancy_Grid_Matching
	//   and Jose Luis Blanco's PhD thesis.
	// ----------------------------------------------------------------------
	internal::se2_l2_impl_return_t<float> implRet;
#if MRPT_ARCH_INTEL_COMPATIBLE
	if (mrpt::cpu::supports(mrpt::cpu::feature::SSE2))
	{ implRet = mrpt::tfest::internal::se2_l2_impl_SSE2(in_correspondences); }
	else
#endif
	{
		implRet = se2_l2_impl(in_correspondences);
	}

	out_transformation.phi = (implRet.Ax != 0 || implRet.Ay != 0)
		? atan2(
			  static_cast<double>(implRet.Ay), static_cast<double>(implRet.Ax))
		: 0.0;

	const double ccos = cos(out_transformation.phi);
	const double csin = sin(out_transformation.phi);

	out_transformation.x =
		implRet.mean_x_a - implRet.mean_x_b * ccos + implRet.mean_y_b * csin;
	out_transformation.y =
		implRet.mean_y_a - implRet.mean_x_b * csin - implRet.mean_y_b * ccos;

	if (out_estimateCovariance)
	{
		CMatrixDouble33* C = out_estimateCovariance;  // less typing!

		// Compute the normalized covariance matrix:
		// -------------------------------------------
		double var_x_a = 0, var_y_a = 0, var_x_b = 0, var_y_b = 0;
		const double N_1_inv = 1.0 / (N - 1);

		// 0) Precompute the unbiased variances estimations:
		// ----------------------------------------------------
		for (const auto& in_correspondence : in_correspondences)
		{
			var_x_a += square(in_correspondence.this_x - implRet.mean_x_a);
			var_y_a += square(in_correspondence.this_y - implRet.mean_y_a);
			var_x_b += square(in_correspondence.other_x - implRet.mean_x_b);
			var_y_b += square(in_correspondence.other_y - implRet.mean_y_b);
		}
		var_x_a *= N_1_inv;	 //  /= (N-1)
		var_y_a *= N_1_inv;
		var_x_b *= N_1_inv;
		var_y_b *= N_1_inv;

		// 1) Compute  BETA = s_Delta^2 / s_p^2
		// --------------------------------
		const double BETA = (var_x_a + var_y_a + var_x_b + var_y_b) *
			pow(static_cast<double>(N), 2.0) * static_cast<double>(N - 1);

		// 2) And the final covariance matrix:
		//  (remember: this matrix has yet to be
		//   multiplied by var_p to be the actual covariance!)
		// -------------------------------------------------------
		const double D = square(implRet.Ax) + square(implRet.Ay);

		(*C)(0, 0) = 2.0 * N_inv +
			BETA *
				square(
					(implRet.mean_x_b * implRet.Ay +
					 implRet.mean_y_b * implRet.Ax) /
					D);
		(*C)(1, 1) = 2.0 * N_inv +
			BETA *
				square(
					(implRet.mean_x_b * implRet.Ax -
					 implRet.mean_y_b * implRet.Ay) /
					D);
		(*C)(2, 2) = BETA / D;

		(*C)(0, 1) = (*C)(1, 0) = -BETA *
			(implRet.mean_x_b * implRet.Ay + implRet.mean_y_b * implRet.Ax) *
			(implRet.mean_x_b * implRet.Ax - implRet.mean_y_b * implRet.Ay) /
			square(D);

		(*C)(0, 2) = (*C)(2, 0) = BETA *
			(implRet.mean_x_b * implRet.Ay + implRet.mean_y_b * implRet.Ax) /
			pow(D, 1.5);

		(*C)(1, 2) = (*C)(2, 1) = BETA *
			(implRet.mean_y_b * implRet.Ay - implRet.mean_x_b * implRet.Ax) /
			pow(D, 1.5);
	}

	return true;

	MRPT_END
}
