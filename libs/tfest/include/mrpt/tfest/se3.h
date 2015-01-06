/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/utils/utils_defs.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/math/CMatrixFixedNumeric.h>
#include <mrpt/utils/TMatchingPair.h>
#include <mrpt/tfest/link_pragmas.h>
#include <mrpt/poses/poses_frwds.h>

namespace mrpt
{
	namespace tfest
	{
		/** \addtogroup mrpt_tfest_grp
		  * @{ */

		/** This function implements the Horn method for computing the change in pose between two coordinate systems
		  * \param[in] inPoints		A vector containing the coordinates of the input points in the format:
		  *							[x11 y11 z11, x12 y12 z12, x21 y21 z21, x22 y22 z22, x31 y31 z31, x32 y32 z32, ...  ]
		  *							where [xi1 yi1 zi1] and [xi2 yi2 zi2] represent the i-th pair of corresponding 3D points in the two coordinate systems "1" and "2"
		  * \param[out] outQuat	A 7D vector containing the traslation and rotation (in a quaternion form) which indicates the change in pose of system "2" wrt "1".
		  * \param[in]  forceScaleToUnity	Whether or not force the scale employed to rotate the coordinate systems to one (rigid transformation)
		  *
		  * \return The computed scale of the optimal transformation (will be 1.0 for a perfectly rigid translation + rotation).
		  * \sa THornMethodOpts
		  */
		double TFEST_IMPEXP HornMethod(
			const std::vector<double>  &inPoints,
			std::vector<double>        &outQuat,
			bool                 forceScaleToUnity = false );

		//! \overload
		double TFEST_IMPEXP HornMethod(
			const std::vector<double>      &inPoints,
			mrpt::poses::CPose3DQuat &outQuat,
			bool                      forceScaleToUnity  = false);

		/** This method provides the closed-form solution of absolute orientation using unit quaternions to a set of over-constrained correspondences for finding the 6D rigid transformation between two cloud of 3D points.
		  *  The output 3D pose is computed using the method described in "Closed-form solution of absolute orientation using unit quaternions", BKP Horn, Journal of the Optical Society of America, 1987.
		  *
		  * \param in_correspondences The set of correspondences in TMatchingPairList form ("this" and "other").
		  * \param out_transformation The change in pose (CPose3DQuat) of the "other" reference system wrt "this" reference system which minimizes the mean-square-error between all the correspondences.
		  * \exception Raises a std::exception if the list "in_correspondences" has not a minimum of three correspondences.
		  * \return True if there are at least three correspondences, or false otherwise, thus we cannot establish any correspondence.
		  *  Implemented by FAMD, 2007. Revised in 2010.
		  * \sa robustRigidTransformation
		  */
		bool TFEST_IMPEXP leastSquareErrorRigidTransformation6D(
			const mrpt::utils::TMatchingPairList	&in_correspondences,
			mrpt::poses::CPose3DQuat							&out_transformation,
			double								&out_scale,
			const bool 							forceScaleToUnity = false );

		/** This method provides the closed-form solution of absolute orientation using unit quaternions to a set of over-constrained correspondences for finding the 6D rigid transformation between two cloud of 3D points.
		  *  The output 3D pose is computed using the method described in "Closed-form solution of absolute orientation using unit quaternions", BKP Horn, Journal of the Optical Society of America, 1987.
		  *
		  * \param in_correspondences The set of correspondences.
		  * \param out_transformation The change in pose (CPose3DQuat) of the "other" reference system wrt "this" reference system which minimizes the mean-square-error between all the correspondences.
		  * \exception Raises a std::exception if the list "in_correspondences" has not a minimum of two correspondences.
		  * \return True if there are at least two correspondences, or false if one or none, thus we cannot establish any correspondence.
		  *  Implemented by FAMD, 2007. Revised in 2010
		  * \sa robustRigidTransformation
		  */
		bool TFEST_IMPEXP leastSquareErrorRigidTransformation6D(
			const mrpt::utils::TMatchingPairList	&in_correspondences,
			mrpt::poses::CPose3D								&out_transformation,
			double								&out_scale,
			const bool 							forceScaleToUnity = false );

		/** This method provides the closed-form solution of absolute orientation using unit quaternions to a set of over-constrained correspondences for finding the 6D rigid transformation between two cloud of 3D points using RANSAC.
		  *  The output 3D pose is computed using the method described in "Closed-form solution of absolute orientation using unit quaternions", BKP Horn, Journal of the Optical Society of America, 1987.
		  *  If supplied, the output covariance matrix is computed using... TODO
		  * \todo Explain covariance!!
		  *
		  * \param in_correspondences The set of correspondences.
		  * \param out_transformation The pose that minimizes the mean-square-error between all the correspondences.
		  * \param out_scale The estimated scale of the rigid transformation (should be very close to 1.0)
		  * \param out_inliers_idx Indexes within the "in_correspondences" list which corresponds with inliers
		  * \param ransac_minSetSize The minimum amount of points in the set
		  * \param ransac_nmaxSimulations The maximum number of iterations of the RANSAC algorithm
		  * \param ransac_maxSetSizePct The (minimum) assumed percent (0.0 - 1.0) of the input set to be considered as inliers
		  * \exception Raises a std::exception if the list "in_correspondences" has not a minimum of two correspondences.
		  * \return True if there are at least two correspondences, or false if one or none, thus we cannot establish any correspondence.
		  *  Implemented by FAMD, 2008.
		  * \sa robustRigidTransformation
		  */
		bool TFEST_IMPEXP leastSquareErrorRigidTransformation6DRANSAC(
			const mrpt::utils::TMatchingPairList	&in_correspondences,
			mrpt::poses::CPose3D								&out_transformation,
			double								&out_scale,
			vector_int							&out_inliers_idx,
			const unsigned int					ransac_minSetSize = 5,
			const unsigned int					ransac_nmaxSimulations = 50,
			const double						ransac_maxSetSizePct = 0.7,
			const bool							forceScaleToUnity = false );


		/** @} */  // end of grouping

	}

} // End of namespace
