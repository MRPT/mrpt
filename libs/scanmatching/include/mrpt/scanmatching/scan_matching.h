/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef ScanMatching_H
#define ScanMatching_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/math/CMatrixFixedNumeric.h>
#include <mrpt/utils/TMatchingPair.h>
#include <mrpt/scanmatching/link_pragmas.h>
#include <mrpt/poses/poses_frwds.h>

namespace mrpt
{
	/** A set of scan matching-related static functions.
	 * \sa mrpt::slam::CICP
	 * \ingroup mrpt_scanmatching_grp
	 */
	namespace scanmatching
	{
		/** \addtogroup mrpt_scanmatching_grp
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
		double SCANMATCHING_IMPEXP HornMethod(
			const std::vector<double>  &inPoints,
			std::vector<double>        &outQuat,
			bool                 forceScaleToUnity = false );

		//! \overload
		double SCANMATCHING_IMPEXP HornMethod(
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
		bool SCANMATCHING_IMPEXP leastSquareErrorRigidTransformation6D(
			const TMatchingPairList	&in_correspondences,
			CPose3DQuat							&out_transformation,
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
		bool SCANMATCHING_IMPEXP leastSquareErrorRigidTransformation6D(
			const TMatchingPairList	&in_correspondences,
			CPose3D								&out_transformation,
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
		bool SCANMATCHING_IMPEXP leastSquareErrorRigidTransformation6DRANSAC(
			const TMatchingPairList	&in_correspondences,
			CPose3D								&out_transformation,
			double								&out_scale,
			vector_int							&out_inliers_idx,
			const unsigned int					ransac_minSetSize = 5,
			const unsigned int					ransac_nmaxSimulations = 50,
			const double						ransac_maxSetSizePct = 0.7,
			const bool							forceScaleToUnity = false );


		/** This method provides the basic least-square-error solution to a set of over-constrained correspondences for finding the (x,y,phi) rigid transformation between two planes.
		  *  The optimal transformation q fulfills:   \f$ point_this = q \oplus point_other \f$
		  * \param in_correspondences The set of correspondences.
		  * \param out_transformation The pose that minimizes the mean-square-error between all the correspondences.
		  * \param out_estimateCovariance If provided (!=NULL) this will contain on return a 3x3 covariance matrix with the NORMALIZED optimal estimate uncertainty. This matrix must be multiplied by \f$\sigma^2_p\f$, the variance of matched points in \f$x\f$ and \f$y\f$ (see paper http://www.mrpt.org/Paper:Occupancy_Grid_Matching)
		  * \exception Raises a std::exception if the list "in_correspondences" has not a minimum of two correspondences.
		  * \return True if there are at least two correspondences, or false if one or none, thus we cannot establish any correspondence.
		  * \sa robustRigidTransformation
		  */
		bool SCANMATCHING_IMPEXP leastSquareErrorRigidTransformation(
			TMatchingPairList	&in_correspondences,
			CPose2D							&out_transformation,
		 mrpt::math::CMatrixDouble33					*out_estimateCovariance = NULL );

		/** This method provides the basic least-square-error solution to a set of over-constrained correspondences for finding the (x,y,phi) rigid transformation between two planes.
		  *  The optimal transformation q fulfills:   \f$ point_this = q \oplus point_other \f$
		  * \param in_correspondences The set of correspondences.
		  * \param out_transformation The pose that minimizes the mean-square-error between all the correspondences.
		  * \param out_estimateCovariance If provided (!=NULL) this will contain on return a 3x3 covariance matrix with the NORMALIZED optimal estimate uncertainty. This matrix must be multiplied by \f$\sigma^2_p\f$, the variance of matched points in \f$x\f$ and \f$y\f$ (see paper http://www.mrpt.org/Paper:Occupancy_Grid_Matching)
		  * \exception Raises a std::exception if the list "in_correspondences" has not a minimum of two correspondences.
		  * \return True if there are at least two correspondences, or false if one or none, thus we cannot establish any correspondence.
		  * \sa robustRigidTransformation
		  */
		bool SCANMATCHING_IMPEXP leastSquareErrorRigidTransformation(
			TMatchingPairList	&in_correspondences,
			CPosePDFGaussian				&out_transformation );

		/** This method implements a RANSAC-based robust estimation of the rigid transformation between two planar frames of references, returning a probability distribution over all the posibilities as a Sum of Gaussians.
		  *
		  *  The technique was described in the paper:
		  *		- J.L. Blanco, J. González-Jimenez and J.A. Fernandez-Madrigal. "A robust, multi-hypothesis approach to matching occupancy grid maps". Robotica, available on CJO2013. doi:10.1017/S0263574712000732. http://journals.cambridge.org/action/displayAbstract?aid=8815308
		  *
		  * This works are follows:
				- Repeat "ransac_nSimulations" times:
					- Randomly pick TWO correspondences from the set "in_correspondences".
					- Compute the associated rigid transformation.
					- For "ransac_maxSetSize" randomly selected correspondences, test for "consensus" with the current group:
						- If if is compatible (ransac_mahalanobisDistanceThreshold), grow the "consensus set"
						- If not, do not add it.
		  *
		  *  For more details refer to the tutorial on <a href="http://www.mrpt.org/Scan_Matching_Algorithms">scan matching methods</a>.
		  *  NOTE:
		  *        - If a pointer is supplied to "out_largestSubSet", the largest consensus sub-set
		  *          of correspondences will be returned there.
		  *        - The parameter "normalizationStd" is the <b>standard deviation</b> (not variance) of landmarks
		  *          being matched in X,Y. Used to normalize covariances returned as the SoG.
		  *        - If ransac_nSimulations=0 then an adaptive algorithm is used to determine the number of iterations, such as
		  *           a good model is found with a probability p=0.999, or that passed as the parameter probability_find_good_model
		  *        - When using "probability_find_good_model", the minimum number of iterations can be set with "ransac_min_nSimulations".
		  *        - "ransac_maxSetSize" should be set to "in_correspondences.size()" to make sure that every correspondence is tested for each random permutation.
		  *
		  *  If ransac_fuseByCorrsMatch=true (the default), the weight of Gaussian modes will be increased when an exact match in the
		  *   subset of correspondences for the modes is found. Otherwise, an approximate method is used as test by just looking at the
		  *   resulting X,Y,PHI means (Threshold in this case are: ransac_fuseMaxDiffXY, ransac_fuseMaxDiffPhi).
		  *
		  * \param[in] max_rmse_to_end Stop searching for solutions when the RMSE of one solution is below this threshold. Special value "0" means "auto", which employs "2*normalizationStd".
		  *
		  * \exception Raises a std::exception if the list "in_correspondences" has not a minimum of two correspondences.
		  * \sa leastSquareErrorRigidTransformation
		  */
		void SCANMATCHING_IMPEXP robustRigidTransformation(
			TMatchingPairList	&in_correspondences,
			poses::CPosePDFSOG				&out_transformation,
			float							normalizationStd,
			unsigned int					ransac_minSetSize = 3,
			unsigned int					ransac_maxSetSize = 20,
			float							ransac_mahalanobisDistanceThreshold = 3.0f,
			unsigned int					ransac_nSimulations = 0,
			TMatchingPairList		*out_largestSubSet = NULL,
			bool						ransac_fuseByCorrsMatch = true,
			float						ransac_fuseMaxDiffXY = 0.01f,
			float						ransac_fuseMaxDiffPhi = mrpt::utils::DEG2RAD(0.1f),
			bool						ransac_algorithmForLandmarks = true,
			double 						probability_find_good_model = 0.999,
			unsigned int				ransac_min_nSimulations = 1500,
			const bool                  verbose = false,
			double                      max_rmse_to_end = 0
			);


		/** @} */  // end of grouping

	}

} // End of namespace

#endif
