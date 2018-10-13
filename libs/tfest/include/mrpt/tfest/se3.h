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
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/tfest/TMatchingPair.h>
#include <mrpt/poses/poses_frwds.h>
#include <mrpt/tfest/indiv-compat-decls.h>

namespace mrpt::tfest
{
/** \addtogroup mrpt_tfest_grp
 * @{ */

/** Least-squares (L2 norm) solution to finding the optimal SE(3) transform
 * between two reference frames using the "quaternion" or Horn's method:
 *  - "Closed-form solution of absolute orientation using unit quaternions",
 * BKP Horn, Journal of the Optical Society of America, 1987.
 *
 *  The optimal transformation `q` fulfills \f$ p_{this} = q \oplus p_{other}
 * \f$, that is, the
 *  transformation of frame `other` with respect to `this`.
 *
 *  \image html tfest_frames.png
 *
 * \param[in]  in_correspondences  The coordinates of the input points for the
 * two coordinate systems "this" and "other"
 * \param[out] out_transform       The output transformation
 * \param[out] out_scale           The computed scale of the optimal
 * transformation (will be 1.0 for a perfectly rigid translation + rotation).
 * \param[in]  forceScaleToUnity   Whether or not force the scale employed to
 * rotate the coordinate systems to one (rigid transformation)
 * \note [New in MRPT 1.3.0] This function replaces
 * mrpt::scanmatching::leastSquareErrorRigidTransformation6DRANSAC() and
 * mrpt::scanmatching::HornMethod()
 * \sa se2_l2, se3_l2_robust
 */
bool se3_l2(
	const mrpt::tfest::TMatchingPairList& in_correspondences,
	mrpt::poses::CPose3DQuat& out_transform, double& out_scale,
	bool forceScaleToUnity = false);

/** \overload
 *
 * This version accepts corresponding points as two vectors of TPoint3D (must
 * have identical length).
 */
bool se3_l2(
	const std::vector<mrpt::math::TPoint3D>& in_points_this,
	const std::vector<mrpt::math::TPoint3D>& in_points_other,
	mrpt::poses::CPose3DQuat& out_transform, double& out_scale,
	bool forceScaleToUnity = false);

/** Parameters for se3_l2_robust(). See function for more details */
struct TSE3RobustParams
{
	/** (Default=5)  The minimum amount of points in a set to start a consensus
	 * set. \sa ransac_maxSetSizePct */
	unsigned int ransac_minSetSize{5};
	/** (Default=50) The maximum number of iterations of the RANSAC algorithm */
	unsigned int ransac_nmaxSimulations{50};
	/** (Default=0.5) The minimum ratio (0.0 - 1.0) of the input set that is
	 * considered to be inliers. *Important*: The minimum size of a consensus
	 * set to be accepted will be "INPUT_CORRESPONDENCES*ransac_maxSetSizePct".
	 */
	double ransac_maxSetSizePct{0.5};
	/** (Default=0.05) The maximum distance in X,Y,Z for a solution to be
	 * considered as matching a candidate solution (In meters) */
	double ransac_threshold_lin{0.05};
	/** (Default=1 deg) The maximum angle (yaw,pitch,roll) for a solution to be
	 * considered as matching a candidate solution (In radians) */
	double ransac_threshold_ang{mrpt::DEG2RAD(1.)};
	/** (Default=0.03) The maximum difference in scale for a solution to be
	 * considered as matching a candidate solution (dimensionless) */
	double ransac_threshold_scale{0.03};
	/** (Default=true)  */
	bool forceScaleToUnity{true};
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
};

/** Output placeholder for se3_l2_robust() */
struct TSE3RobustResult
{
	/** The best transformation found */
	mrpt::poses::CPose3DQuat transformation;
	/** The estimated scale of the rigid transformation (should be very close to
	 * 1.0) */
	double scale{.0};
	/** Indexes within the `in_correspondences` list which corresponds with
	 * inliers */
	std::vector<uint32_t> inliers_idx;
};

/** Least-squares (L2 norm) solution to finding the optimal SE(3) transform
 * between two reference frames using RANSAC and the "quaternion" or Horn's
 * method:
 *  - "Closed-form solution of absolute orientation using unit quaternions",
 * BKP Horn, Journal of the Optical Society of America, 1987.
 *
 *  The optimal transformation `q` fulfills \f$ p_{this} = q \oplus p_{other}
 * \f$, that is, the
 *  transformation of frame `other` with respect to `this`.
 *
 *  \image html tfest_frames.png
 *
 * \param[in] in_correspondences The set of correspondences.
 * \param[in] in_params Method parameters (see docs for TSE3RobustParams)
 * \param[out] out_results Results: transformation, scale, etc.
 *
 * \return True if the minimum number of correspondences was found, false
 * otherwise.
 * \note Implemented by FAMD, 2008. Re-factored by JLBC, 2015.
 * \note [New in MRPT 1.3.0] This function replaces
 * mrpt::scanmatching::leastSquareErrorRigidTransformation6DRANSAC()
 * \sa se2_l2, se3_l2
 */
bool se3_l2_robust(
	const mrpt::tfest::TMatchingPairList& in_correspondences,
	const TSE3RobustParams& in_params, TSE3RobustResult& out_results);

/** @} */  // end of grouping
}  // namespace mrpt::tfest
