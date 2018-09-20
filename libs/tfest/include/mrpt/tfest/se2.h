/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/math/math_frwds.h>
#include <mrpt/math/CMatrixFixedNumeric.h>
#include <mrpt/poses/CPosePDFSOG.h>
#include <mrpt/tfest/TMatchingPair.h>
#include <mrpt/poses/poses_frwds.h>
#include <mrpt/tfest/indiv-compat-decls.h>

namespace mrpt
{
/** Functions for estimating the optimal transformation between two frames of
 * references given measurements of corresponding points.
 * \sa mrpt::slam::CICP
 * \ingroup mrpt_tfest_grp
 */
namespace tfest
{
/** \addtogroup mrpt_tfest_grp
 * @{ */

/** Least-squares (L2 norm) solution to finding the optimal SE(2) (x,y,yaw)
 * between two reference frames.
 *  The optimal transformation `q` fulfills \f$ p_{this} = q \oplus p_{other}
 * \f$, that is, the
 *  transformation of frame `other` with respect to `this`.
 *
 *  \image html tfest_frames.png
 *
 * \param[in] in_correspondences The set of correspondences.
 * \param[out] out_transformation The pose that minimizes the mean-square-error
 * between all the correspondences.
 * \param[out] out_estimateCovariance If provided (!=nullptr) this will contain
 * on return a 3x3 covariance matrix with the NORMALIZED optimal estimate
 * uncertainty. This matrix must be multiplied by \f$\sigma^2_p\f$, the variance
 * of matched points in \f$x\f$ and \f$y\f$ (see paper
 * http://www.mrpt.org/Paper:Occupancy_Grid_Matching)
 * \return True if there are at least two correspondences, or false if one or
 * none, thus we cannot establish any correspondence.
 * \sa robustRigidTransformation
 *
 * \note Reference for covariance calculation: J.L. Blanco, J.
 * Gonzalez-Jimenez, J.A. Fernandez-Madrigal, "A Robust, Multi-Hypothesis
 * Approach to Matching Occupancy Grid Maps", Robotica, 2013.
 * http://dx.doi.org/10.1017/S0263574712000732
 * \note [New in MRPT 1.3.0] This function replaces
 * mrpt::scanmatching::leastSquareErrorRigidTransformation()
 * \note This function is hand-optimized for SSE2 architectures (if SSE2 is
 * enabled from CMake)
 * \sa se3_l2, se2_l2_robust
 * \ingroup sse_optimizations
 * \ingroup mrpt_tfest_grp
 */
bool se2_l2(
	const mrpt::tfest::TMatchingPairList& in_correspondences,
	mrpt::math::TPose2D& out_transformation,
	mrpt::math::CMatrixDouble33* out_estimateCovariance = nullptr);

/** \overload */
bool se2_l2(
	const mrpt::tfest::TMatchingPairList& in_correspondences,
	mrpt::poses::CPosePDFGaussian& out_transformation);

/** Parameters for se2_l2_robust(). See function for more details */
struct TSE2RobustParams
{
	/** (Default=3) */
	unsigned int ransac_minSetSize{3};
	/** (Default = 20) */
	unsigned int ransac_maxSetSize{20};
	/** (Default = 3.0) */
	double ransac_mahalanobisDistanceThreshold{3.0};
	/** (Default = 0) If set to 0, an adaptive algorithm is used to determine
	 * the number of iterations, such as a good model is found with a
	 * probability p=0.999, or that passed as the parameter
	 * probability_find_good_model */
	unsigned int ransac_nSimulations{0};
	/** (Default = true)  If true, the weight of Gaussian modes will be
	 * increased when an exact match in the
	 *   subset of correspondences for the modes is found. Otherwise, an
	 * approximate method is used as test by just looking at the
	 *   resulting X,Y,PHI means. Threshold in this case are:
	 * ransac_fuseMaxDiffXY, ransac_fuseMaxDiffPhi */
	bool ransac_fuseByCorrsMatch{true};
	/** (Default = 0.01) */
	double ransac_fuseMaxDiffXY{0.01};
	/** (Default=0.1degree) (In radians) */
	double ransac_fuseMaxDiffPhi{mrpt::DEG2RAD(0.1)};
	/** (Default = true) Use Mahalanobis distance (true) or Euclidean dist
	 * (false) */
	bool ransac_algorithmForLandmarks{true};
	/** (Default = 0.999) See parameter ransac_nSimulations. When using
	 * `probability_find_good_model`, the minimum number of iterations can be
	 * set with `ransac_min_nSimulations` */
	double probability_find_good_model{0.999};
	/** (Default = 1500) See parameter probability_find_good_model */
	unsigned int ransac_min_nSimulations{1500};
	/** Stop searching for solutions when the RMSE of one solution is below this
	 * threshold. Special value "0" means "auto", which employs
	 * "2*normalizationStd". */
	double max_rmse_to_end{0};
	/** (Default=false) */
	bool verbose{false};

	/** If provided, this user callback will be invoked to determine the
	 * individual compatibility between each potential pair
	 * of elements. Can check image descriptors, geometrical properties, etc.
	 * \return Must return true if the pair is a potential match, false
	 * otherwise.
	 */
	// std::function<bool(TPotentialMatch)>  user_individual_compat_callback; //
	// This could be used in the future when we enforce C++11 to users...
	TFunctorCheckPotentialMatch user_individual_compat_callback;
	/** User data to be passed to user_individual_compat_callback() */
	void* user_individual_compat_callback_userdata;

	/** Default values */
	TSE2RobustParams() = default;
};

/** Output placeholder for se2_l2_robust() */
struct TSE2RobustResult
{
	/** The output as a set of transformations (sum of Gaussians) */
	mrpt::poses::CPosePDFSOG transformation;
	/** the largest consensus sub-set */
	mrpt::tfest::TMatchingPairList largestSubSet;
	/** Number of actual iterations executed  */
	unsigned int ransac_iters{0};

	TSE2RobustResult() = default;
};

/** Robust least-squares (L2 norm) solution to finding the optimal SE(2)
 * (x,y,yaw) between two reference frames.
 * This method implements a RANSAC-based robust estimation, returning a
 * probability distribution over all the posibilities as a Sum of Gaussians.
 *
 *  The optimal transformation `q` fulfills \f$ p_{this} = q \oplus p_{other}
 * \f$, that is, the
 *  transformation of frame `other` with respect to `this`.
 *
 *  \image html tfest_frames.png
 *
 *  The technique was described in the paper:
 *    - J.L. Blanco, J. Gonzalez-Jimenez, J.A. Fernandez-Madrigal, "A Robust,
 * Multi-Hypothesis Approach to Matching Occupancy Grid Maps", Robotica, 2013.
 * http://dx.doi.org/10.1017/S0263574712000732
 *
 * This works as follows:
 * - Repeat "ransac_nSimulations" times:
 * 	- Randomly pick TWO correspondences from the set "in_correspondences".
 * 	- Compute the associated rigid transformation.
 * 	- For "ransac_maxSetSize" randomly selected correspondences, test for
 * "consensus" with the current group:
 * 		- If if is compatible (ransac_mahalanobisDistanceThreshold), grow
 * the "consensus set"
 * 		- If not, do not add it.
 *
 *  For more details refer to the tutorial on <a
 * href="http://www.mrpt.org/Scan_Matching_Algorithms">scan matching
 * methods</a>.
 * \param[in] in_normalizationStd The <b>standard deviation</b> (not variance)
 * of landmarks/points/features being matched in X,Y. Used to normalize
 * covariances returned as the SoG. (Refer to paper)
 *
 * <b>NOTE</b>: Parameter `ransac_maxSetSize` should be set to
 * `in_correspondences.size()` to make sure that every correspondence is tested
 * for each random permutation.
 *
 * \return True upon success, false if no subset was found with the minimum
 * number of correspondences.
 * \note [New in MRPT 1.3.0] This function replaces
 * mrpt::scanmatching::robustRigidTransformation()
 * \sa se3_l2, se2_l2_robust
 */
bool se2_l2_robust(
	const mrpt::tfest::TMatchingPairList& in_correspondences,
	const double in_normalizationStd, const TSE2RobustParams& in_ransac_params,
	TSE2RobustResult& out_results);

/** @} */  // end of grouping
}  // namespace tfest
}  // namespace mrpt
