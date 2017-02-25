/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

// ------------------------------------------------------
// This test file is a subset of a complete example. See: 
//   [MRPT]/samples/ransac-data-association/
// ------------------------------------------------------

#include <mrpt/random.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPosePDFSOG.h>
#include <mrpt/math/geometry.h>
#include <mrpt/tfest/se2.h>
#include <mrpt/maps/CSimplePointsMap.h>

#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::random;
using namespace mrpt::maps;
using namespace mrpt::poses;
using namespace std;

// ============= PARAMETERS ===================
const size_t NUM_OBSERVATIONS_TO_SIMUL = 15; 
const size_t RANSAC_MINIMUM_INLIERS    = 9;  // Min. # of inliers to accept

const float normalizationStd = 0.10f; // 1 sigma noise (meters)
const float ransac_mahalanobisDistanceThreshold = 5.0f;
const size_t MINIMUM_RANSAC_ITERS = 100000;

const size_t NUM_MAP_FEATS = 50;
const double MAP_SIZE_X    = 30;
const double MAP_SIZE_Y    = 15;
// ==============================================

struct TObs
{
	size_t ID; // Ground truth ID
	double x,y;
};

// Return true if test succeds. 
// Due to RANSAC being non-deterministic, there exists a small chance of failed tests even if the algorithm is ok.
bool ransac_data_assoc_run()
{
	randomGenerator.randomize(); // randomize with time
	// --------------------------------
	// Load feature map:
	// --------------------------------
	CSimplePointsMap  the_map;
	// Generate random MAP:
	the_map.resize(NUM_MAP_FEATS);
	for (size_t i=0;i<NUM_MAP_FEATS;i++)
	{
		the_map.setPoint(i, 
			randomGenerator.drawUniform(0,MAP_SIZE_X),
			randomGenerator.drawUniform(0,MAP_SIZE_Y)
			);
	}
	const size_t nMapPts = the_map.size();
	const size_t nObs=NUM_OBSERVATIONS_TO_SIMUL;

	// Read the observations themselves:
	vector<TObs> observations;
	observations.resize(nObs);

	const mrpt::poses::CPose2D  GT_pose(
		mrpt::random::randomGenerator.drawUniform(-10,10+MAP_SIZE_X),
		mrpt::random::randomGenerator.drawUniform(-10,10+MAP_SIZE_Y),
		mrpt::random::randomGenerator.drawUniform(-M_PI,M_PI) );

	const mrpt::poses::CPose2D  GT_pose_inv = -GT_pose;

	std::vector<std::pair<size_t,float> > idxs;
	the_map.kdTreeRadiusSearch2D(GT_pose.x(),GT_pose.y(), 1000, idxs);
	ASSERT_(idxs.size()>=nObs)

	for (size_t i=0;i<nObs;i++)
	{
		double gx,gy;
		the_map.getPoint(idxs[i].first, gx,gy);

		double lx,ly;
		GT_pose_inv.composePoint(gx,gy, lx,ly);

		observations[i].ID = idxs[i].first;
		observations[i].x = lx + mrpt::random::randomGenerator.drawGaussian1D(0,normalizationStd);
		observations[i].y = ly + mrpt::random::randomGenerator.drawGaussian1D(0,normalizationStd);
	}

	// ----------------------------------------------------
	// Generate list of individual-compatible pairings
	// ----------------------------------------------------
	TMatchingPairList all_correspondences;

	all_correspondences.reserve(nMapPts*nObs);

	// ALL possibilities: 
	for (size_t j=0;j<nObs;j++)
	{
		TMatchingPair match;
		match.other_idx = j;
		match.other_x = observations[j].x;
		match.other_y = observations[j].y;

		for (size_t i=0;i<nMapPts;i++) {
			match.this_idx = i;
			the_map.getPoint(i, match.this_x, match.this_y );
			all_correspondences.push_back(match);
		}
	}

	// ----------------------------------------------------
	//  Run RANSAC-based D-A
	// ----------------------------------------------------
	mrpt::tfest::TSE2RobustParams params;
	mrpt::tfest::TSE2RobustResult results;

	params.ransac_minSetSize = RANSAC_MINIMUM_INLIERS;     // ransac_minSetSize (to add the solution to the SOG)
	params.ransac_maxSetSize = all_correspondences.size(); // ransac_maxSetSize: Test with all data points
	params.ransac_mahalanobisDistanceThreshold = ransac_mahalanobisDistanceThreshold;
	params.ransac_nSimulations = 0; // 0=auto
	params.ransac_fuseByCorrsMatch = true;
	params.ransac_fuseMaxDiffXY = 0.01f;
	params.ransac_fuseMaxDiffPhi = DEG2RAD(0.1);
	params.ransac_algorithmForLandmarks = true;
	params.probability_find_good_model = 0.999999;
	params.ransac_min_nSimulations  = MINIMUM_RANSAC_ITERS; // (a lower limit to the auto-detected value of ransac_nSimulations)
	params.verbose = false;

	// Run ransac data-association:
	mrpt::tfest::se2_l2_robust(all_correspondences, normalizationStd, params, results);

	//mrpt::poses::CPosePDFSOG  & best_poses  = results.transformation;
	TMatchingPairList         & out_best_pairings = results.largestSubSet;

	// Reconstruct the SE(2) transformation for these pairings:
	mrpt::poses::CPosePDFGaussian  solution_pose;
	mrpt::tfest::se2_l2(out_best_pairings, solution_pose);

	// Normalized covariance: scale!
	solution_pose.cov *= square(normalizationStd);

	if (!(solution_pose.mean.distanceTo(GT_pose) < 0.9 && std::abs(solution_pose.mean.phi()-GT_pose.phi())<DEG2RAD(10)))
	{
		std::cerr 
		<< "Solution pose: " << solution_pose.mean << endl
		<< "Ground truth pose: " << GT_pose << endl;
		return false;
	}
	return true;
}


TEST(tfest, ransac_data_assoc)
{
	// Run randomized experiments:
	bool any_ok = false;
	for (int i=0;i<3;i++)
		if (ransac_data_assoc_run())
			any_ok = true;

	EXPECT_TRUE(any_ok);
}
