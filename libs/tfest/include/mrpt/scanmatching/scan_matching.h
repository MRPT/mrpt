/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once

#ifndef MRPT_SCANMATCHING_SUPRESS_BACKCOMPAT_WARNING
#	include <mrpt/utils/mrpt_macros.h>
	MRPT_WARNING("Deprecated header: Use <mrpt/tfest.h> or individual headers instead")
#endif

#include <mrpt/utils/utils_defs.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/math/CMatrixFixedNumeric.h>
#include <mrpt/utils/TMatchingPair.h>
#include <mrpt/poses/poses_frwds.h>
#include <mrpt/tfest/link_pragmas.h>

namespace mrpt 
{
	namespace  scanmatching
	{
		/** \deprecated Use functions in mrpt::tfest */
		MRPT_DEPRECATED("Deprecated: Use mrpt::tfest::se3_l2() instead")
		double TFEST_IMPEXP HornMethod(
			const std::vector<double>  &inPoints,
			std::vector<double>        &outQuat,
			bool                 forceScaleToUnity = false);

		/** \deprecated Use functions in mrpt::tfest */
		MRPT_DEPRECATED("Deprecated: Use mrpt::tfest::se3_l2() instead")
		double TFEST_IMPEXP HornMethod(
			const std::vector<double>      &inPoints,
			mrpt::poses::CPose3DQuat &outQuat,
			bool                      forceScaleToUnity = false);

		/** \deprecated Use functions in mrpt::tfest */
		MRPT_DEPRECATED("Deprecated: Use mrpt::tfest::se3_l2() instead")
		bool TFEST_IMPEXP leastSquareErrorRigidTransformation6D(
			const mrpt::utils::TMatchingPairList	&in_correspondences,
			mrpt::poses::CPose3DQuat							&out_transformation,
			double								&out_scale,
			const bool 							forceScaleToUnity = false);

		/** \deprecated Use functions in mrpt::tfest */
		MRPT_DEPRECATED("Deprecated: Use mrpt::tfest::se3_l2() instead")
		bool TFEST_IMPEXP leastSquareErrorRigidTransformation6D(
			const mrpt::utils::TMatchingPairList	&in_correspondences,
			mrpt::poses::CPose3D								&out_transformation,
			double								&out_scale,
			const bool 							forceScaleToUnity = false);

		/** \deprecated Use functions in mrpt::tfest */
		MRPT_DEPRECATED("Deprecated: Use mrpt::tfest::se3_l2_robust() instead")
		bool TFEST_IMPEXP leastSquareErrorRigidTransformation6DRANSAC(
			const mrpt::utils::TMatchingPairList	&in_correspondences,
			mrpt::poses::CPose3D								&out_transformation,
			double								&out_scale,
			vector_int							&out_inliers_idx,
			const unsigned int					ransac_minSetSize = 5,
			const unsigned int					ransac_nmaxSimulations = 50,
			const double						ransac_maxSetSizePct = 0.7,
			const bool							forceScaleToUnity = false);

		/** \deprecated Use functions in mrpt::tfest */
		MRPT_DEPRECATED("Deprecated: Use mrpt::tfest::se2_l2() instead")
		bool TFEST_IMPEXP leastSquareErrorRigidTransformation(
			mrpt::utils::TMatchingPairList	&in_correspondences,
			mrpt::poses::CPose2D							&out_transformation,
			mrpt::math::CMatrixDouble33					*out_estimateCovariance = NULL);

		/** \deprecated Use functions in mrpt::tfest */
		MRPT_DEPRECATED("Deprecated: Use mrpt::tfest::se2_l2() instead")
		bool TFEST_IMPEXP leastSquareErrorRigidTransformation(
			mrpt::utils::TMatchingPairList	&in_correspondences,
			mrpt::poses::CPosePDFGaussian				&out_transformation );

		/** \deprecated Use functions in mrpt::tfest */
		MRPT_DEPRECATED("Deprecated: Use mrpt::tfest::se2_l2_robust() instead")
		void TFEST_IMPEXP robustRigidTransformation(
			mrpt::utils::TMatchingPairList	&in_correspondences,
			mrpt::poses::CPosePDFSOG				&out_transformation,
			float							normalizationStd,
			unsigned int					ransac_minSetSize = 3,
			unsigned int					ransac_maxSetSize = 20,
			float							ransac_mahalanobisDistanceThreshold = 3.0f,
			unsigned int					ransac_nSimulations = 0,
			mrpt::utils::TMatchingPairList		*out_largestSubSet = NULL,
			bool						ransac_fuseByCorrsMatch = true,
			float						ransac_fuseMaxDiffXY = 0.01f,
			float						ransac_fuseMaxDiffPhi = mrpt::utils::DEG2RAD(0.1f),
			bool						ransac_algorithmForLandmarks = true,
			double 						probability_find_good_model = 0.999,
			unsigned int				ransac_min_nSimulations = 1500,
			const bool                  verbose = false,
			double                      max_rmse_to_end = 0
			);
	}
}
