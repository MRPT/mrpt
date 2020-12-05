/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "tfest-precomp.h"  // Precompiled headers

#include <mrpt/tfest/se3.h>

#include <mrpt/core/round.h>
#include <mrpt/math/utils.h>  // linspace()
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/Lie/SO.h>
#include <mrpt/random.h>
#include <iostream>
#include <numeric>

using namespace mrpt;
using namespace mrpt::tfest;
using namespace mrpt::random;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;

/*---------------------------------------------------------------
						 se3_l2_robust
  ---------------------------------------------------------------*/
bool tfest::se3_l2_robust(
	const mrpt::tfest::TMatchingPairList& in_correspondences,
	const TSE3RobustParams& params, TSE3RobustResult& results)
{
	MRPT_START

	const size_t nCorrs = in_correspondences.size();

	// -------------------------------------------
	// RANSAC parameters
	// -------------------------------------------
	// Minimum error achieved so far
	double min_err = std::numeric_limits<double>::max();
	size_t max_size = 0;  // Maximum size of the consensus set so far
	double scale;  // Output scale

	// Minimum number of points to fit the model
	const size_t n = params.ransac_minSetSize;
	// Minimum number of points to be considered a good set
	const size_t d = mrpt::round(nCorrs * params.ransac_maxSetSizePct);
	// Maximum number of iterations
	const size_t maxIters = params.ransac_nmaxSimulations;

	ASSERTMSG_(
		d >= n,
		"Minimum number of points to be considered a good set is < Minimum "
		"number of points to fit the model");

	// -------------------------------------------
	// MAIN loop
	// -------------------------------------------
	for (size_t iterations = 0; iterations < maxIters; iterations++)
	{
		if (params.verbose)
			std::cout << "[tfest::se3_l2_robust] Iteration " << (iterations + 1)
					  << "/" << maxIters << "\n";

		// Generate maybe inliers
		const auto rub = mrpt::math::linspace<uint32_t>(0, nCorrs - 1, nCorrs);
		const auto mbSet = getRandomGenerator().permuteVector(rub);

		std::vector<uint32_t> cSet;  // consensus set

		// Compute first inliers output
		TMatchingPairList mbInliers;
		mbInliers.reserve(n);
		for (size_t i = 0; mbInliers.size() < n && i < nCorrs; i++)
		{
			const size_t idx = mbSet[i];

			// User-provided filter:
			if (params.user_individual_compat_callback)
			{
				mrpt::tfest::TPotentialMatch pm;
				pm.idx_this = in_correspondences[idx].this_idx;
				pm.idx_other = in_correspondences[idx].other_idx;
				if (!params.user_individual_compat_callback(pm))
					continue;  // Skip this one!
			}

			mbInliers.push_back(in_correspondences[idx]);
			cSet.push_back(idx);
		}

		// Check minimum number:
		if (cSet.size() < n)
		{
			if (params.verbose)
				std::cerr << "[tfest::se3_l2_robust] Iter " << iterations
						  << ": It was not possible to find the min no of "
							 "(compatible) matching pairs.\n";
			continue;  // Try again
		}

		CPose3DQuat mbOutQuat;
		bool res = mrpt::tfest::se3_l2(
			mbInliers, mbOutQuat, scale, params.forceScaleToUnity);
		if (!res)
		{
			std::cerr << "[tfest::se3_l2_robust] tfest::se3_l2() returned "
						 "false for tentative subset during RANSAC "
						 "iteration!\n";
			continue;
		}

		// Maybe inliers Output
		const CPose3D mbOut = CPose3D(mbOutQuat);
		CVectorDouble mbOut_vec(7);
		mbOut_vec[0] = mbOut.x();
		mbOut_vec[1] = mbOut.y();
		mbOut_vec[2] = mbOut.z();

		mbOut_vec[3] = mbOut.yaw();
		mbOut_vec[4] = mbOut.pitch();
		mbOut_vec[5] = mbOut.roll();

		mbOut_vec[6] = scale;

		// Inner loop: for each point NOT in the maybe inliers
		for (size_t k = n; k < nCorrs; k++)
		{
			const size_t idx = mbSet[k];

			// User-provided filter:
			if (params.user_individual_compat_callback)
			{
				mrpt::tfest::TPotentialMatch pm;
				pm.idx_this = in_correspondences[idx].this_idx;
				pm.idx_other = in_correspondences[idx].other_idx;
				if (!params.user_individual_compat_callback(pm))
					continue;  // Skip this one!
			}

			// Consensus set: Maybe inliers + new point
			CPose3DQuat csOutQuat;
			mbInliers.push_back(in_correspondences[idx]);  // Insert
			res = mrpt::tfest::se3_l2(
				mbInliers, csOutQuat, scale, params.forceScaleToUnity);
			mbInliers.erase(mbInliers.end() - 1);  // Erase

			if (!res)
			{
				std::cerr << "[tfest::se3_l2_robust] tfest::se3_l2() returned "
							 "false for tentative subset during RANSAC "
							 "iteration!\n";
				continue;
			}

			// Is this point a supporter of the initial inlier group?
			const CPose3D csOut = CPose3D(csOutQuat);

			const double linDist = mbOut.distanceTo(csOut);
			const double angDist = mrpt::poses::Lie::SO<3>::log(
									   (csOut - mbOut).getRotationMatrix())
									   .norm();
			const double scaleDist = std::abs(mbOut_vec[6] - scale);

			if (linDist < params.ransac_threshold_lin &&
				angDist < params.ransac_threshold_ang &&
				scaleDist < params.ransac_threshold_scale)
			{
				// Inlier detected -> add to the inlier list
				cSet.push_back(idx);
			}
			else
			{
				// cout << " It " << iterations << " - RANSAC Outlier Detected:
				// " << k << endl;
			}
		}  // end 'inner' for

		// Test cSet size
		if (cSet.size() >= d)
		{
			// Good set of points found
			TMatchingPairList cSetInliers;
			cSetInliers.resize(cSet.size());
			for (size_t m = 0; m < cSet.size(); m++)
				cSetInliers[m] = in_correspondences[cSet[m]];

			// Compute output: Consensus Set + Initial Inliers Guess
			CPose3DQuat cIOutQuat;
			res = mrpt::tfest::se3_l2(
				cSetInliers, cIOutQuat, scale,
				params.forceScaleToUnity);  // Compute output
			ASSERTMSG_(
				res,
				"tfest::se3_l2() returned false for tentative subset during "
				"RANSAC iteration!");

			// Compute error for consensus_set
			const CPose3D cIOut = CPose3D(cIOutQuat);
			const double err = std::sqrt(
				square(mbOut_vec[0] - cIOut.x()) +
				square(mbOut_vec[1] - cIOut.y()) +
				square(mbOut_vec[2] - cIOut.z()) +
				square(mbOut_vec[3] - cIOut.yaw()) +
				square(mbOut_vec[4] - cIOut.pitch()) +
				square(mbOut_vec[5] - cIOut.roll()) +
				square(mbOut_vec[6] - scale));

			// Is the best set of points so far?
			if (err < min_err && cSet.size() >= max_size)
			{
				min_err = err;
				max_size = cSet.size();
				results.transformation = cIOutQuat;
				results.scale = scale;
				results.inliers_idx = cSet;
			}
			// printf(" - Consensus set size: %u - Error: %.6f\n", (unsigned
			// int)cSet.size(), err );
		}  // end if cSet.size() > d
		else
		{
			// printf(" - Consensus set size: %u - Not big enough!\n", (unsigned
			// int)cSet.size() );
		}
	}  // end 'iterations' for

	if (max_size == 0)
	{
		if (params.verbose)
			std::cerr
				<< "[se3_l2_robust] No solution found, maximum size is == 0!\n";
		return false;
	}

	return true;
	MRPT_END
}
