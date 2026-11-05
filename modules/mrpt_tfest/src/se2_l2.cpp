/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

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
    const TMatchingPairList& in_correspondences, CPosePDFGaussian& out_transformation)
{
  mrpt::math::TPose2D p;
  const bool ret = tfest::se2_l2(in_correspondences, p, &out_transformation.cov);
  out_transformation.mean = CPose2D(p);
  return ret;
}

// Non-vectorized version
namespace
{
mrpt::tfest::internal::se2_l2_impl_return_t<double> se2_l2_impl(
    const TMatchingPairList& in_correspondences)
{
  const size_t N = in_correspondences.size();
  ASSERT_(N >= 2);
  const double N_inv = 1.0 / static_cast<double>(N);

  // Both point sets are shifted by a pivot (the first pair) and accumulated
  // in double precision, to avoid cancellation errors for points far from
  // the origin. Ax and Ay are invariant to these shifts.
  const double x0a = in_correspondences[0].global.x;
  const double y0a = in_correspondences[0].global.y;
  const double x0b = in_correspondences[0].local.x;
  const double y0b = in_correspondences[0].local.y;

  double SumXa = 0;
  double SumXb = 0;
  double SumYa = 0;
  double SumYb = 0;
  double Sxx = 0;
  double Sxy = 0;
  double Syx = 0;
  double Syy = 0;

  for (const auto& p : in_correspondences)
  {
    // Get the pair of points in the correspondence:
    const double xa = static_cast<double>(p.global.x) - x0a;
    const double ya = static_cast<double>(p.global.y) - y0a;
    const double xb = static_cast<double>(p.local.x) - x0b;
    const double yb = static_cast<double>(p.local.y) - y0b;

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

  mrpt::tfest::internal::se2_l2_impl_return_t<double> ret;
  ret.mean_x_a = x0a + SumXa * N_inv;
  ret.mean_y_a = y0a + SumYa * N_inv;
  ret.mean_x_b = x0b + SumXb * N_inv;
  ret.mean_y_b = y0b + SumYb * N_inv;

  // Auxiliary variables Ax,Ay:
  ret.Ax = static_cast<double>(N) * (Sxx + Syy) - SumXa * SumXb - SumYa * SumYb;
  ret.Ay = SumXa * SumYb + static_cast<double>(N) * (Syx - Sxy) - SumXb * SumYa;

  return ret;
}
}  // namespace

/*---------------------------------------------------------------
      leastSquareErrorRigidTransformation

   Compute the best transformation (x,y,phi) given a set of
  correspondences between 2D points in two different maps.
   This method is intensively used within ICP.
  ---------------------------------------------------------------*/
bool tfest::se2_l2(
    const TMatchingPairList& in_correspondences,
    TPose2D& out_transformation,
    CMatrixDouble33* out_estimateCovariance)
{
  MRPT_START

  const size_t N = in_correspondences.size();

  if (N < 2)
  {
    return false;
  }

  const double N_inv = 1.0 / static_cast<double>(N);

  // ----------------------------------------------------------------------
  // Compute the estimated pose. Notation from the paper:
  // "Mobile robot motion estimation by 2d scan matching with genetic and
  // iterative
  // closest point algorithms", J.L. Martinez, J. Gonzalez, J. Morales,
  // A. Mandow, A.J. Garcia-Cerezo, Journal of Field Robotics, 23(1):21-34,
  // 2006.
  // ----------------------------------------------------------------------

  // ----------------------------------------------------------------------
  //  For the formulas of the covariance, see:
  //   https://www.mrpt.org/paper-occupancy-grid-matching/
  //   and Jose Luis Blanco's PhD thesis.
  // ----------------------------------------------------------------------
  internal::se2_l2_impl_return_t<double> implRet;
#if MRPT_ARCH_INTEL_COMPATIBLE
  if (mrpt::cpu::supports(mrpt::cpu::feature::SSE2))
  {
    implRet = mrpt::tfest::internal::se2_l2_impl_SSE2(in_correspondences);
  }
  else
#endif
  {
    implRet = se2_l2_impl(in_correspondences);
  }

  out_transformation.phi =
      (implRet.Ax != 0 || implRet.Ay != 0) ? atan2(implRet.Ay, implRet.Ax) : 0.0;

  const double ccos = cos(out_transformation.phi);
  const double csin = sin(out_transformation.phi);

  out_transformation.x = implRet.mean_x_a - implRet.mean_x_b * ccos + implRet.mean_y_b * csin;
  out_transformation.y = implRet.mean_y_a - implRet.mean_x_b * csin - implRet.mean_y_b * ccos;

  if (out_estimateCovariance)
  {
    CMatrixDouble33* C = out_estimateCovariance;  // less typing!

    // Compute the normalized covariance matrix:
    // -------------------------------------------
    double var_x_a = 0;
    double var_y_a = 0;
    double var_x_b = 0;
    double var_y_b = 0;
    const double N_1_inv = 1.0 / static_cast<double>(N - 1);

    // 0) Precompute the unbiased variances estimations:
    // ----------------------------------------------------
    for (const auto& in_correspondence : in_correspondences)
    {
      var_x_a += square(in_correspondence.global.x - implRet.mean_x_a);
      var_y_a += square(in_correspondence.global.y - implRet.mean_y_a);
      var_x_b += square(in_correspondence.local.x - implRet.mean_x_b);
      var_y_b += square(in_correspondence.local.y - implRet.mean_y_b);
    }
    var_x_a *= N_1_inv;  //  /= (N-1)
    var_y_a *= N_1_inv;
    var_x_b *= N_1_inv;
    var_y_b *= N_1_inv;

    // 1) Compute  BETA = s_Delta^2 / s_p^2
    // --------------------------------
    const double BETA = (var_x_a + var_y_a + var_x_b + var_y_b) * pow(static_cast<double>(N), 2.0) *
                        static_cast<double>(N - 1);

    // 2) And the final covariance matrix:
    //  (remember: this matrix has yet to be
    //   multiplied by var_p to be the actual covariance!)
    // -------------------------------------------------------
    const double D = square(implRet.Ax) + square(implRet.Ay);
    if (D == 0)
    {
      // Degenerate geometry (e.g. all local points coincide): the rotation
      // and its uncertainty are undefined.
      return false;
    }

    (*C)(0, 0) = 2.0 * N_inv +
                 BETA * square((implRet.mean_x_b * implRet.Ay + implRet.mean_y_b * implRet.Ax) / D);
    (*C)(1, 1) = 2.0 * N_inv +
                 BETA * square((implRet.mean_x_b * implRet.Ax - implRet.mean_y_b * implRet.Ay) / D);
    (*C)(2, 2) = BETA / D;

    (*C)(0, 1) = (*C)(1, 0) =
        -BETA * (implRet.mean_x_b * implRet.Ay + implRet.mean_y_b * implRet.Ax) *
        (implRet.mean_x_b * implRet.Ax - implRet.mean_y_b * implRet.Ay) / square(D);

    (*C)(0, 2) = (*C)(2, 0) =
        BETA * (implRet.mean_x_b * implRet.Ay + implRet.mean_y_b * implRet.Ax) / pow(D, 1.5);

    (*C)(1, 2) = (*C)(2, 1) =
        BETA * (implRet.mean_y_b * implRet.Ay - implRet.mean_x_b * implRet.Ax) / pow(D, 1.5);
  }

  return true;

  MRPT_END
}
